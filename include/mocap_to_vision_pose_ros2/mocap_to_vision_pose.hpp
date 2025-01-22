#include "builtin_interfaces/msg/time.hpp"
#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mavros_msgs/srv/command_home.hpp"
#include "optitrack_multiplexer_ros2_msgs/msg/rigid_body_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

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

  // set home position
  void SetHomePosition();

  // publish gp origin
  void PublishGPOrigin();

  // callback for mocap data
  void MocapCallback(
      const optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped::SharedPtr);

  /*--------- member variables -----------*/
  // topic on which we get the data from mocap
  std::string mocap_topic_;
  // frame in which to send the coordinates (header.frame_id)
  std::string frame_id_;
  // position variance
  double pos_var_;
  // attitude variance (radians for quaternions)
  double att_var_;
  // origin for the ekf2 and for home
  std::vector<double> origin_;
  // subscriber to the mocap data
  rclcpp::Subscription<optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>::
      SharedPtr mocap_sub_;
  // publisher to the mavros topic
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      mavros_pub_;
  // client to set the home position
  rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr command_home_client_;
  // publisher to set the gp origin
  rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr
      gp_origin_pub_;
};

} // namespace mocap_to_vision_pose
