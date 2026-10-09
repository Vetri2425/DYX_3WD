
#include <atomic>
#include <csignal>
#include <cstdio>
#include <memory>

#include "dyx3_system_gateway/gateway_node.hpp"

int main(int argc, char** argv) {
  // A write to a socket whose peer has gone must surface as EPIPE, never kill the process: the
  // launch file shuts the whole control graph down when this node exits (GW-002).
  std::signal(SIGPIPE, SIG_IGN);
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<dyx3_gateway::GatewayNode>());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "system_gateway: %s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
