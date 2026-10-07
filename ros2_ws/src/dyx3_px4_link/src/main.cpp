#include <sys/mman.h>

#include <cstdio>
#include <memory>

#include "dyx3_px4_link/px4_link_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Real-time discipline: no page faults on the tick path. Failure is reported, not fatal
  // (containers and CI often forbid it).
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::fprintf(stderr, "px4_link_node: mlockall failed (continuing unlocked)\n");
  }
  auto node = std::make_shared<dyx3_px4_link::Px4LinkNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
