#include "dyx3_px4_link/msg_version_handshake.hpp"

namespace dyx3_px4_link {

Handshake::Handshake(std::vector<HandshakeTopic> topics, double retry_s) : retry_s_(retry_s) {
  e_.reserve(topics.size());
  for (auto& t : topics) {
    Entry en;
    en.t = std::move(t);
    if (en.t.definition_missing) {
      en.s = HandshakeState::Mismatch;
      if (mismatch_reason_.empty()) {
        mismatch_reason_ = "no installed .msg definition for " + en.t.request_name;
      }
    }
    e_.push_back(std::move(en));
  }
}

std::vector<size_t> Handshake::due_requests(double now_s) {
  std::vector<size_t> due;
  for (size_t i = 0; i < e_.size(); ++i) {
    if (e_[i].s != HandshakeState::Pending) continue;
    if (now_s - e_[i].last_sent_s >= retry_s_) {
      e_[i].last_sent_s = now_s;
      due.push_back(i);
    }
  }
  return due;
}

void Handshake::on_response(const std::string& topic_name, bool success, uint32_t hash) {
  for (auto& en : e_) {
    if (en.t.request_name != topic_name) continue;
    if (en.s != HandshakeState::Pending) return;  // Ok stays Ok; Mismatch is permanent
    if (!success) {
      en.s = HandshakeState::Mismatch;
      mismatch_reason_ = "firmware does not know topic " + topic_name;
    } else if (hash != en.t.expected_hash) {
      en.s = HandshakeState::Mismatch;
      mismatch_reason_ = "format hash mismatch for " + topic_name + ": firmware " +
                         std::to_string(hash) + ", installed px4_msgs " +
                         std::to_string(en.t.expected_hash);
    } else {
      en.s = HandshakeState::Ok;
    }
    return;
  }
}

void Handshake::rearm() {
  ++generation_;
  mismatch_reason_.clear();
  for (auto& en : e_) {
    en.last_sent_s = -1.0e18;
    if (en.t.definition_missing) {
      en.s = HandshakeState::Mismatch;
      mismatch_reason_ = "no installed .msg definition for " + en.t.request_name;
    } else {
      en.s = HandshakeState::Pending;
    }
  }
}

HandshakeState Handshake::state() const {
  bool all_ok = true;
  for (const auto& en : e_) {
    if (en.s == HandshakeState::Mismatch) return HandshakeState::Mismatch;
    if (en.s != HandshakeState::Ok) all_ok = false;
  }
  return all_ok && !e_.empty() ? HandshakeState::Ok : HandshakeState::Pending;
}

size_t Handshake::pending_count() const {
  size_t n = 0;
  for (const auto& en : e_) n += en.s == HandshakeState::Pending ? 1U : 0U;
  return n;
}

}  // namespace dyx3_px4_link
