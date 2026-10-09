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
  // The heartbeat is bound to the connection that sent it (GW-001): when that connection goes,
  // the heartbeat goes with it, even if another client is still connected.
  void note_heartbeat(double now_s, int client) {
    have_ = true;
    last_ = now_s;
    client_ = client;
  }
  int client() const { return client_; }  // the client that sent the last heartbeat; -1 if none
  // alive iff the client that sent the last heartbeat is still connected AND that heartbeat
  // arrived within the timeout.
  OperatorLinkState state(double now_s, bool heartbeat_client_connected) const {
    OperatorLinkState s;
    if (!have_) return s;
    s.age_s = now_s > last_ ? now_s - last_ : 0.0;
    s.alive = heartbeat_client_connected && s.age_s <= timeout_s_;
    return s;
  }

private:
  double timeout_s_;
  bool have_{false};
  double last_{0.0};
  int client_{-1};
};

}  // namespace dyx3_gateway
