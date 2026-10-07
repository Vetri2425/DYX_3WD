// operator_link — the tablet heartbeat timeout the Jetson owns (architecture 4.3.1). Contract:
// docs/contracts/dyx3_system_gateway.md section 4. Pure C++, clock-injected.
#pragma once

namespace dyx3_gateway {

struct OperatorLinkState {
  bool alive{false};
  double age_s{0.0};  // seconds since the last heartbeat; 0 when never heard
};

class OperatorLink {
public:
  explicit OperatorLink(double timeout_s) : timeout_s_(timeout_s) {}
  void note_heartbeat(double now_s) {
    have_ = true;
    last_ = now_s;
  }
  // alive iff a client is connected AND a heartbeat arrived within the timeout.
  OperatorLinkState state(double now_s, int clients) const {
    OperatorLinkState s;
    if (!have_) return s;
    s.age_s = now_s > last_ ? now_s - last_ : 0.0;
    s.alive = clients > 0 && s.age_s <= timeout_s_;
    return s;
  }

private:
  double timeout_s_;
  bool have_{false};
  double last_{0.0};
};

}  // namespace dyx3_gateway
