#include <memory>

#include "dyx3_gnss_rtk/rtk_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dyx3_gnss_rtk::RtkNode>());
  rclcpp::shutdown();
  return 0;
}
