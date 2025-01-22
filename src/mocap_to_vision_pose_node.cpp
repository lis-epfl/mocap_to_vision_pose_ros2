#include "mocap_to_vision_pose.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mocap_to_vision_pose::MocapToVisionPose>());
  rclcpp::shutdown();
}
