#ifndef MOCAP_TO_VISION_POSE_CLASS_H_
#define MOCAP_TO_VISION_POSE_CLASS_H_

#include "builtin_interfaces/msg/time.hpp"
#include "optitrack_multiplexer_ros2_msgs/msg/rigid_body_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <Eigen/Dense>
namespace mocap_to_vision_pose {

class MocapToVisionPose : public rclcpp::Node {
public:
  // constructor
  MocapToVisionPose();

private:
  /*-------------- methods ---------------*/
  // declare ros parameters
  void DeclareRosParameters();

  // initialize ros parameters
  void InitializeRosParameters();

  // callback for mocap data
  void MocapCallback(
      const optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped::SharedPtr);

  /*--------- member variables -----------*/
  // topic on which we get the data from mocap
  std::string mocap_topic_;
  // frame in which to send the coordinates (header.frame_id)
  std::string frame_id_;
  // position variance
  Eigen::Vector2f pos_var_;
  // attitude variance (radians for quaternions)
  Eigen::Vector3f att_var_;
  // subscriber to the mocap data
  rclcpp::Subscription<optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>::
      SharedPtr mocap_sub_;
  // px4 ros2 local position measurement interface
  std::shared_ptr<px4_ros2::LocalPositionMeasurementInterface> position_interface_;
};

} // namespace mocap_to_vision_pose

#endif
