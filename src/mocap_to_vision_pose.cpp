#include "mocap_to_vision_pose.hpp"

namespace mocap_to_vision_pose {

MocapToVisionPose::MocapToVisionPose()
    : rclcpp::Node("mocap_to_vision_pose_node") {
  // declare environment parameters
  DeclareRosParameters();

  // initialize parameters
  InitializeRosParameters();

  // create px4 ros2 local position measurement interface
  // using LocalNED frame for both pose and velocity (matches OptiTrack
  // convention)
  position_interface_ =
      std::make_shared<px4_ros2::LocalPositionMeasurementInterface>(
          *this, px4_ros2::PoseFrame::LocalNED,
          px4_ros2::VelocityFrame::LocalNED);

  // subscription to the mocap topic
  mocap_sub_ = create_subscription<
      optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>(
      mocap_topic_, 10,
      std::bind(&MocapToVisionPose::MocapCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Using mocap topic: %s", mocap_topic_.c_str());

  RCLCPP_INFO(
      get_logger(),
      "MoCap to Vision Pose node initialized with PX4 native ROS2 interface");

  pi_about_x_quat_ = Eigen::Quaternionf(Eigen::AngleAxisf(EIGEN_PI, Eigen::Vector3f::UnitX()));
}

void MocapToVisionPose::DeclareRosParameters() {
  declare_parameter("mocap_topic",
                    "/optitrack_multiplexer_node/rigid_body/mavros");
  declare_parameter("frame_id", "base_link");
  declare_parameter("pos_var", 0.000001);
  declare_parameter("att_var", 0.000001);
}

void MocapToVisionPose::InitializeRosParameters() {
  mocap_topic_ = get_parameter("mocap_topic").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  auto pos_var = get_parameter("pos_var").as_double();
  pos_var_ = Eigen::Vector2f::Constant(pos_var);
  auto att_var = get_parameter("att_var").as_double();
  att_var_ = Eigen::Vector3f::Constant(att_var);
}

void MocapToVisionPose::MocapCallback(
    const optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped::SharedPtr
        msg) {
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[THROTTLED] Executing mocap callback with position: [%5.2f, %5.2f, %5.2f]", msg->rigid_body.pose.position.x, msg->rigid_body.pose.position.y, msg->rigid_body.pose.position.z);
  if (!msg->rigid_body.tracking_valid) {
    // do not publish an invalid message
    return;
  }

  // create local position measurement
  px4_ros2::LocalPositionMeasurement measurement{};

  measurement.timestamp_sample = msg->stamp;

  // negative value for FLU -> FRD
  measurement.position_xy = Eigen::Vector2f(-msg->rigid_body.pose.position.y,
                                            -msg->rigid_body.pose.position.x);
  measurement.position_z = -msg->rigid_body.pose.position.z; // - because FLU -> FRD

  // set position variance (same for all axes)
  measurement.position_xy_variance = pos_var_;
  measurement.position_z_variance = pos_var_(0); // assuming everything is the same

  auto attitude_quaternion = Eigen::Quaternionf(-msg->rigid_body.pose.orientation.q_w,
                         msg->rigid_body.pose.orientation.q_y,
                         msg->rigid_body.pose.orientation.q_x,
                         msg->rigid_body.pose.orientation.q_z);

  // set orientation (quaternion)
  measurement.attitude_quaternion = attitude_quaternion.normalized(); // FLU -> FRD


  // set orientation variance
  measurement.attitude_variance = att_var_;

  // send measurement to PX4
  position_interface_->update(measurement);
}
}  // namespace mocap_to_vision_pose

