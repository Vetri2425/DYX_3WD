// msg_version_handshake — loud px4_msgs format handshake. See docs/contracts/dyx3_px4_link.md
// section 6. Pure C++: the node feeds it responses and asks which requests are due.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace dyx3_px4_link {

struct HandshakeTopic {
  std::string request_name;  // base topic name sent in the request, e.g. "/fmu/in/vehicle_command"
  uint32_t expected_hash{0};
  bool definition_missing{false};  // the installed px4_msgs had no .msg for it: permanent mismatch
};

enum class HandshakeState : uint8_t { Pending = 0, Ok, Mismatch };

class Handshake {
public:
  Handshake(std::vector<HandshakeTopic> topics, double retry_s);

  // Indices of topics whose request should be (re)sent now. Marks them as sent at now_s.
  std::vector<size_t> due_requests(double now_s);

  // Response from /fmu/out/message_format_response. `topic_name` is the echoed char[50] (up to the
  // first NUL). Unknown topics are ignored (another client's response). Once Mismatch, always
  // Mismatch for this armed generation: a later matching response does not clear it.
  void on_response(const std::string& topic_name, bool success, uint32_t hash);

  // A session reset re-arms every topic to Pending and clears a Mismatch: a different firmware
  // may be on the other side. Returns nothing; call after StalenessMonitor::consume_reset().
  void rearm();

  HandshakeState state() const;  // Mismatch if any mismatched, Ok if all Ok, else Pending
  size_t pending_count() const;
  const std::string& first_mismatch_reason() const { return mismatch_reason_; }
  uint32_t generation() const { return generation_; }

private:
  struct Entry {
    HandshakeTopic t;
    HandshakeState s{HandshakeState::Pending};
    double last_sent_s{-1.0e18};
  };
  std::vector<Entry> e_;
  double retry_s_;
  std::string mismatch_reason_;
  uint32_t generation_{0};
};

}  // namespace dyx3_px4_link
