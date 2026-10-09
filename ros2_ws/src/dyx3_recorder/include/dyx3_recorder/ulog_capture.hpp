// ulog_capture — reassembles the FCU's streamed ULog into the run directory. Contract:
// docs/contracts/dyx3_recorder.md section 3. Pure C++ (std only).
//
// The FCU starts its ULog stream once (LOGGING_START at link-up): the 16-byte file header and the
// definitions section (B, F, I, M, P, Q messages) are sent only at the start of the stream, the
// subscriptions (A) right after them. A run opens later, mid-stream. So the capture is fed EVERY
// chunk, run or not, and keeps in memory what a reader needs to decode the data section (REC-005):
//   * the file header + the definitions section (until the first message of another type),
//   * every 'A' (add subscription) not removed by an 'R', and every data-section 'P' change.
// A run file starts with that cache and then continues with whole messages only: the stream is cut
// into messages (3-byte header: uint16 size, uint8 type), resynchronising after a gap at the
// chunk's first_message_offset. If no stream start was seen (the recorder started after the FCU
// stream), the file has no header and header_status() says so.
#pragma once

#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace dyx3_recorder {

struct UlogGap {
  uint16_t expected_seq;
  uint16_t got_seq;
  uint32_t missing_chunks;
  uint64_t file_offset;   // byte offset in the reassembled file where the gap is
  uint8_t resync_offset;  // first_message_offset of the chunk after the gap
};

class UlogCapture {
public:
  enum class HeaderState { kNone, kDefinitions, kComplete, kLost, kTooLarge };
  static constexpr size_t kMaxHeaderBytes = 8U << 20;  // cache bound (definitions + params)

  ~UlogCapture() { close(); }
  // Creates `path` (parent directory must exist) and writes the cached header if there is one.
  // False on failure.
  bool open(const std::string& path);
  // Flush, fsync and close. False if any of them failed (also recorded in write_failed()).
  bool close();
  bool is_open() const { return f_ != nullptr; }

  // Feed every chunk of the stream, with or without an open file. Returns true when bytes were
  // appended to the open file.
  bool on_chunk(uint16_t seq, uint8_t first_message_offset, const uint8_t* data, size_t size);

  HeaderState header_state() const { return header_state_; }  // of the stream (the cache)
  size_t cached_header_bytes() const { return header_.size(); }
  // This run's file: "complete" when it starts with a ULog header, otherwise "incomplete: ...".
  std::string header_status() const;
  bool run_header_complete() const { return run_header_status_ == "complete"; }

  uint64_t bytes() const { return bytes_; }
  uint64_t chunks() const { return chunks_; }
  uint64_t duplicates() const { return duplicates_; }
  uint64_t out_of_order() const { return out_of_order_; }
  uint64_t parse_errors() const { return parse_errors_; }
  uint32_t segments() const { return segments_; }  // files of this run (a stream restart rolls)
  bool write_failed() const { return write_failed_; }
  const std::vector<UlogGap>& gaps() const { return gaps_; }
  std::string gaps_json() const;

private:
  void stream_start(const uint8_t* file_header);
  void feed(const uint8_t* data, size_t size);
  void on_message(const uint8_t* m, size_t n);
  void write(const void* p, size_t n);
  bool open_file(const std::string& path);
  void write_cached_header();

  std::FILE* f_{nullptr};
  std::string path_;           // first segment of the run
  uint64_t file_bytes_{0};     // bytes in the current segment

  // stream state (persists across runs)
  bool have_last_{false};
  uint16_t last_seq_{0};
  bool synced_{false};
  std::vector<uint8_t> partial_;
  HeaderState header_state_{HeaderState::kNone};
  std::string header_;                          // file header + definitions
  std::map<uint16_t, std::string> subs_;        // msg_id -> 'A' message
  std::map<std::string, std::string> params_;   // key -> latest data-section 'P' message

  // per run
  std::string run_header_status_{"no file"};
  uint64_t bytes_{0}, chunks_{0}, duplicates_{0}, out_of_order_{0}, parse_errors_{0};
  uint32_t segments_{0};
  bool write_failed_{false};
  std::vector<UlogGap> gaps_;
};

const char* ulog_header_state_name(UlogCapture::HeaderState s);

}  // namespace dyx3_recorder
