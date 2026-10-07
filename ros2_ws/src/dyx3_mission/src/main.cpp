// dyx3_mission entry point. Mission is control-adjacent but NOT real-time: a normal executor is
// fine.
#include <memory>

#include "dyx3_mission/mission_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<dyx3_mission::MissionNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("dyx3_mission"), "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
