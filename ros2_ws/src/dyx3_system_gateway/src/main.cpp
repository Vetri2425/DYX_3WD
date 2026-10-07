
#include <atomic>
#include <csignal>
#include <cstdio>
#include <memory>

#include "dyx3_system_gateway/gateway_node.hpp"

int main(int argc, char** argv) {
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
