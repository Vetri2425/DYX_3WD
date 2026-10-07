// ulog_capture — reassembles the FCU's streamed ULog into the run directory. Contract:
// docs/contracts/dyx3_recorder.md section 3. Pure C++ (std only).
#pragma once

#include <cstdint>
#include <cstdio>
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
  ~UlogCapture() { close(); }
  // Creates `path` (parent directory must exist). False on failure.
  bool open(const std::string& path);
  void close();
  bool is_open() const { return f_ != nullptr; }

  // Returns true when the chunk's bytes were appended.
  bool on_chunk(uint16_t seq, uint8_t first_message_offset, const uint8_t* data, size_t size);

  uint64_t bytes() const { return bytes_; }
  uint64_t chunks() const { return chunks_; }
  uint64_t duplicates() const { return duplicates_; }
  uint64_t out_of_order() const { return out_of_order_; }
  bool write_failed() const { return write_failed_; }
  const std::vector<UlogGap>& gaps() const { return gaps_; }
  std::string gaps_json() const;

private:
  std::FILE* f_{nullptr};
  bool have_last_{false};
  uint16_t last_seq_{0};
  uint64_t bytes_{0}, chunks_{0}, duplicates_{0}, out_of_order_{0};
  bool write_failed_{false};
  std::vector<UlogGap> gaps_;
};

}  // namespace dyx3_recorder
