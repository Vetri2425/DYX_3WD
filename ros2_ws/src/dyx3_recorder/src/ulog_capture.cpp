#include "dyx3_recorder/ulog_capture.hpp"

#include "dyx3_recorder/run_manifest.hpp"

namespace dyx3_recorder {

bool UlogCapture::open(const std::string& path) {
  close();
  f_ = std::fopen(path.c_str(), "wb");
  have_last_ = false;
  bytes_ = chunks_ = duplicates_ = out_of_order_ = 0;
  write_failed_ = false;
  gaps_.clear();
  return f_ != nullptr;
}

void UlogCapture::close() {
  if (f_ != nullptr) {
    std::fflush(f_);
    std::fclose(f_);
    f_ = nullptr;
  }
}

bool UlogCapture::on_chunk(uint16_t seq, uint8_t first_message_offset, const uint8_t* data,
                           size_t size) {
  if (f_ == nullptr) return false;
  if (have_last_) {
    if (seq == last_seq_) {
      ++duplicates_;
      return false;
    }
    const uint16_t expected = static_cast<uint16_t>(last_seq_ + 1);
    const uint16_t ahead = static_cast<uint16_t>(seq - expected);
    if (ahead >= 0x8000U) {  // a "negative" jump: an old chunk arriving late
      ++out_of_order_;
      return false;
    }
    if (ahead != 0) gaps_.push_back(UlogGap{expected, seq, ahead, bytes_, first_message_offset});
  }
  if (size > 0 && std::fwrite(data, 1, size, f_) != size) write_failed_ = true;
  std::fflush(f_);
  bytes_ += size;
  ++chunks_;
  last_seq_ = seq;
  have_last_ = true;
  return !write_failed_;
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
  top.integer("chunks", static_cast<int64_t>(chunks_))
      .integer("bytes", static_cast<int64_t>(bytes_))
      .integer("duplicates_dropped", static_cast<int64_t>(duplicates_))
      .integer("out_of_order_dropped", static_cast<int64_t>(out_of_order_))
      .boolean("write_failed", write_failed_)
      .raw("gaps", a + (gaps_.empty() ? "]" : "\n]"));
  return top.dump() + "\n";
}

}  // namespace dyx3_recorder
