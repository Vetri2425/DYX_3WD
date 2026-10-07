#include <sys/mman.h>

#include <cstdio>
#include <memory>

#include "dyx3_motion_guard/motion_guard_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::fprintf(stderr, "motion_guard_node: mlockall failed (continuing unlocked)\n");
  }
  rclcpp::spin(std::make_shared<dyx3_motion_guard::MotionGuardNode>());
  rclcpp::shutdown();
  return 0;
}
