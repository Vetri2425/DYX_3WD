#include "dyx3_recorder/ulog_capture.hpp"

#include <unistd.h>

#include <cstring>

#include "dyx3_recorder/run_manifest.hpp"

namespace dyx3_recorder {
namespace {

constexpr size_t kFileHeaderLen = 16;  // magic[8] + uint64 timestamp
constexpr size_t kMsgHeaderLen = 3;    // uint16 msg_size + uint8 msg_type
const uint8_t kMagic[7] = {'U', 'L', 'o', 'g', 0x01, 0x12, 0x35};

bool is_definition(uint8_t t) {
  return t == 'B' || t == 'F' || t == 'I' || t == 'M' || t == 'P' || t == 'Q';
}
// A chunk that starts with the ULog magic at first_message_offset 0 starts a stream. It cannot be
// confused with a message: a message starting with "UL" would have type 'o', which does not exist.
bool is_stream_start(uint8_t fmo, const uint8_t* data, size_t size) {
  return fmo == 0 && size >= kFileHeaderLen && std::memcmp(data, kMagic, sizeof kMagic) == 0;
}

bool is_known_type(uint8_t t) {
  return is_definition(t) || t == 'A' || t == 'R' || t == 'D' || t == 'L' || t == 'C' ||
         t == 'S' || t == 'O';
}

}  // namespace

const char* ulog_header_state_name(UlogCapture::HeaderState s) {
  switch (s) {
    case UlogCapture::HeaderState::kNone:
      return "incomplete: no header (the FCU stream started before the recorder)";
    case UlogCapture::HeaderState::kDefinitions:
      return "definitions in progress";
    case UlogCapture::HeaderState::kComplete:
      return "complete";
    case UlogCapture::HeaderState::kLost:
      return "incomplete: header lost (gap in the definitions section)";
    case UlogCapture::HeaderState::kTooLarge:
      return "incomplete: header larger than the cache bound";
  }
  return "unknown";
}

bool UlogCapture::open_file(const std::string& path) {
  f_ = std::fopen(path.c_str(), "wbe");  // e: O_CLOEXEC, the bag child must not inherit it
  file_bytes_ = 0;
  if (f_ != nullptr) ++segments_;
  return f_ != nullptr;
}

bool UlogCapture::open(const std::string& path) {
  close();
  path_ = path;
  bytes_ = chunks_ = duplicates_ = out_of_order_ = parse_errors_ = 0;
  segments_ = 0;
  write_failed_ = false;
  gaps_.clear();
  if (!open_file(path)) {
    run_header_status_ = "no file";
    return false;
  }
  if (header_state_ == HeaderState::kComplete || header_state_ == HeaderState::kDefinitions) {
    write_cached_header();
    run_header_status_ = "complete";
  } else {
    run_header_status_ = ulog_header_state_name(header_state_);
  }
  return true;
}

void UlogCapture::write_cached_header() {
  // Header + definitions, then the subscription table and the parameter changes of the data
  // section so far: everything a reader needs to decode the data messages that follow.
  write(header_.data(), header_.size());
  for (const auto& [id, m] : subs_) write(m.data(), m.size());
  for (const auto& [key, m] : params_) write(m.data(), m.size());
}

bool UlogCapture::close() {
  bool ok = true;
  if (f_ != nullptr) {
    // REC-011: the run's ULog is on disk when the summary says it is.
    ok = std::fflush(f_) == 0;
    ok = (::fsync(fileno(f_)) == 0) && ok;
    ok = (std::fclose(f_) == 0) && ok;
    f_ = nullptr;
    if (!ok) write_failed_ = true;
  }
  return ok;
}

std::string UlogCapture::header_status() const { return run_header_status_; }

void UlogCapture::write(const void* p, size_t n) {
  if (f_ == nullptr || n == 0) return;
  if (std::fwrite(p, 1, n, f_) != n) write_failed_ = true;
  bytes_ += n;
  file_bytes_ += n;
}

void UlogCapture::stream_start(const uint8_t* file_header) {
  header_.assign(reinterpret_cast<const char*>(file_header), kFileHeaderLen);
  subs_.clear();
  params_.clear();
  partial_.clear();
  synced_ = true;
  header_state_ = HeaderState::kDefinitions;
  if (f_ == nullptr) return;
  if (file_bytes_ > 0) {
    // A second stream (px4_link re-sent LOGGING_START) cannot continue a file that already has a
    // header or data: the run rolls to <name>_<n>.ulg.
    close();
    std::string base = path_;
    if (base.size() > 4 && base.compare(base.size() - 4, 4, ".ulg") == 0)
      base.resize(base.size() - 4);
    if (!open_file(base + "_" + std::to_string(segments_ + 1) + ".ulg")) {
      write_failed_ = true;
      return;
    }
  } else if (run_header_status_ != "complete") {
    run_header_status_ = "complete";  // nothing written yet: this file starts with the header
  }
  write(file_header, kFileHeaderLen);
}

bool UlogCapture::on_chunk(uint16_t seq, uint8_t first_message_offset, const uint8_t* data,
                           size_t size) {
  const uint64_t before = bytes_;
  if (have_last_) {
    if (seq == last_seq_) {
      if (f_ != nullptr) ++duplicates_;
      return false;
    }
    const uint16_t expected = static_cast<uint16_t>(last_seq_ + 1);
    const uint16_t ahead = static_cast<uint16_t>(seq - expected);
    const bool restart = is_stream_start(first_message_offset, data, size);
    if (ahead >= 0x8000U && !restart) {  // a "negative" jump: an old chunk arriving late
      if (f_ != nullptr) ++out_of_order_;
      return false;
    }
    if (ahead != 0 && !restart) {
      if (f_ != nullptr)
        gaps_.push_back(UlogGap{expected, seq, ahead, bytes_, first_message_offset});
      synced_ = false;  // the message in progress is lost; resync at first_message_offset
      partial_.clear();
      if (header_state_ == HeaderState::kDefinitions) header_state_ = HeaderState::kLost;
    }
  }
  last_seq_ = seq;
  have_last_ = true;
  if (f_ != nullptr) ++chunks_;

  size_t off = 0;
  if (is_stream_start(first_message_offset, data, size)) {
    stream_start(data);
    off = kFileHeaderLen;
  } else if (!synced_) {
    if (first_message_offset == 255 || first_message_offset >= size) return false;
    synced_ = true;
    partial_.clear();
    off = first_message_offset;
  }
  feed(data + off, size - off);
  return bytes_ > before && !write_failed_;
}

void UlogCapture::feed(const uint8_t* data, size_t size) {
  partial_.insert(partial_.end(), data, data + size);
  size_t pos = 0;
  while (partial_.size() - pos >= kMsgHeaderLen) {
    const uint8_t* m = partial_.data() + pos;
    const size_t total = kMsgHeaderLen + (static_cast<size_t>(m[0]) | (static_cast<size_t>(m[1]) << 8));
    if (!is_known_type(m[2])) {
      // Not a message boundary: drop everything until the next chunk that starts a message.
      ++parse_errors_;
      synced_ = false;
      partial_.clear();
      if (header_state_ == HeaderState::kDefinitions) header_state_ = HeaderState::kLost;
      return;
    }
    if (partial_.size() - pos < total) break;
    on_message(m, total);
    pos += total;
  }
  partial_.erase(partial_.begin(), partial_.begin() + static_cast<std::ptrdiff_t>(pos));
}

void UlogCapture::on_message(const uint8_t* m, size_t n) {
  const uint8_t type = m[2];
  if (header_state_ == HeaderState::kDefinitions) {
    if (is_definition(type)) {
      header_.append(reinterpret_cast<const char*>(m), n);
      if (header_.size() > kMaxHeaderBytes) {
        header_state_ = HeaderState::kTooLarge;
        header_.clear();
      }
      write(m, n);
      return;
    }
    header_state_ = HeaderState::kComplete;  // first message of the data section
  }
  if (type == 'A' && n >= 6) {
    subs_[static_cast<uint16_t>(m[4] | (m[5] << 8))].assign(reinterpret_cast<const char*>(m), n);
  } else if (type == 'R' && n >= 5) {
    subs_.erase(static_cast<uint16_t>(m[3] | (m[4] << 8)));
  } else if (type == 'P' && n >= 4 && 4U + m[3] <= n) {
    params_[std::string(reinterpret_cast<const char*>(m) + 4, m[3])].assign(
        reinterpret_cast<const char*>(m), n);
  }
  write(m, n);
}

std::string UlogCapture::gaps_json() const {
  std::string a = "[";
  for (size_t i = 0; i < gaps_.size(); ++i) {
    const auto& g = gaps_[i];
    JsonObject o;
    o.integer("expected_seq", g.expected_seq)
        .integer("got_seq", g.got_seq)
        .integer("missing_chunks", g.missing_chunks)
        .integer("file_offset", static_cast<int64_t>(g.file_offset))
        .integer("resync_first_message_offset", g.resync_offset);
    a += (i ? ",\n" : "\n") + o.dump(2, 1);
  }
  JsonObject top;
  top.str("header", run_header_status_)
      .integer("segments", segments_)
      .integer("chunks", static_cast<int64_t>(chunks_))
      .integer("bytes", static_cast<int64_t>(bytes_))
      .integer("duplicates_dropped", static_cast<int64_t>(duplicates_))
      .integer("out_of_order_dropped", static_cast<int64_t>(out_of_order_))
      .integer("parse_errors", static_cast<int64_t>(parse_errors_))
      .boolean("write_failed", write_failed_)
      .raw("gaps", a + (gaps_.empty() ? "]" : "\n]"));
  return top.dump() + "\n";
}

}  // namespace dyx3_recorder
