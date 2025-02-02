#include "mocap_to_vision_pose.hpp"

namespace mocap_to_vision_pose {

MocapToVisionPose::MocapToVisionPose()
    : rclcpp::Node("mocap_to_vision_pose_node") {

  // declare environment parameters
  DeclareRosParameters();

  // initialize parameters
  InitializeRosParameters();

  // create a client for the CommandHome service and set home position for the
  // safety features (return to home, geofence ...)
  command_home_client_ =
      create_client<mavros_msgs::srv::CommandHome>("/mavros/cmd/set_home");
  SetHomePosition();

  // create a publisher for the /mavros/global_position/set_gp_origin topic and
  // publish the origin for ekf2
  gp_origin_pub_ = create_publisher<geographic_msgs::msg::GeoPointStamped>(
      "/mavros/global_position/set_gp_origin", 10);
  PublishGPOrigin();

  // publisher to the mavros odometry input topic
  mavros_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/mavros/vision_pose/pose_cov", 10);

  // subscription to the mocap topic
  mocap_sub_ = create_subscription<
      optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>(
      mocap_topic_, 10,
      std::bind(&MocapToVisionPose::MocapCallback, this,
                std::placeholders::_1));
}

void MocapToVisionPose::DeclareRosParameters() {
  declare_parameter("mocap_topic",
                    "/optitrack_multiplexer_node/rigid_body/mavros");
  declare_parameter("frame_id", "base_link");
  declare_parameter("pos_var", 0.000001);
  declare_parameter("att_var", 0.000001);
  declare_parameter("origin", ::std::vector<double>(3, 0.0));
}

void MocapToVisionPose::InitializeRosParameters() {
  mocap_topic_ = get_parameter("mocap_topic").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  pos_var_ = get_parameter("pos_var").as_double();
  att_var_ = get_parameter("att_var").as_double();
  origin_ = get_parameter("origin").as_double_array();
}

void MocapToVisionPose::SetHomePosition() {
  // wait until the service is available
  while (rclcpp::ok() &&
         !command_home_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_INFO(get_logger(), "Waiting for /mavros/cmd/set_home service...");
  }

  bool success = false;
  // prepare the request
  auto request = std::make_shared<mavros_msgs::srv::CommandHome::Request>();
  // set to true to use current GPS position
  request->current_gps = false;

  request->yaw = 0.0;
  request->latitude = origin_[0];
  request->longitude = origin_[1];
  request->altitude = origin_[2];

  // asynchronous service call
  auto result_future = command_home_client_->async_send_request(request);

  // handle the response
  rclcpp::spin_until_future_complete(get_node_base_interface(), result_future);
  if (result_future.wait_for(std::chrono::seconds(2)) ==
      std::future_status::ready) {
    auto response = result_future.get();
    if (response->success) {
      RCLCPP_INFO(get_logger(), "Successfully set home position: %d",
                  response->result);
    } else {
      RCLCPP_ERROR(
          get_logger(),
          "Failed to set home position, try relaunching the launch file: %d",
          response->result);
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Service call timed out.");
  }
}

void MocapToVisionPose::PublishGPOrigin() {
  // Publish the EKF origin
  auto gp_origin_msg = geographic_msgs::msg::GeoPointStamped();
  gp_origin_msg.header.stamp = get_clock()->now();
  gp_origin_msg.header.frame_id = "map";

  gp_origin_msg.position.latitude = origin_[0];
  gp_origin_msg.position.longitude = origin_[1];
  gp_origin_msg.position.altitude = origin_[2];

  gp_origin_pub_->publish(gp_origin_msg);

  // add a small delay to ensure the message is processed
  RCLCPP_INFO(get_logger(),
              "Published EKF origin to /mavros/global_position/set_gp_origin");
}

void MocapToVisionPose::MocapCallback(
    const optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped::SharedPtr
        msg) {

  auto transformed_pose_cov = geometry_msgs::msg::PoseWithCovarianceStamped();

  // correct only for latency_ms
  double latency_seconds =
      static_cast<double>(msg->latency_ms) / 1000.0; // Convert ms to seconds
  rclcpp::Time corrected_stamp =
      rclcpp::Time(msg->stamp) -
      rclcpp::Duration::from_seconds(latency_seconds);

  // update the header of the transformed pose
  transformed_pose_cov.header.stamp = corrected_stamp;

  // set the frame_id
  transformed_pose_cov.header.frame_id = frame_id_;

  // transform the position
  transformed_pose_cov.pose.pose.position.x = msg->rigid_body.pose.position.x;
  transformed_pose_cov.pose.pose.position.y = msg->rigid_body.pose.position.y;
  transformed_pose_cov.pose.pose.position.z = msg->rigid_body.pose.position.z;

  // transform the orientation
  transformed_pose_cov.pose.pose.orientation.x =
      msg->rigid_body.pose.orientation.q_x;
  transformed_pose_cov.pose.pose.orientation.y =
      msg->rigid_body.pose.orientation.q_y;
  transformed_pose_cov.pose.pose.orientation.z =
      msg->rigid_body.pose.orientation.q_z;
  transformed_pose_cov.pose.pose.orientation.w =
      msg->rigid_body.pose.orientation.q_w;

  // set covariance (position and orientation)
  std::vector<double> covariance(
      36, 0.0); // 6x6 covariance matrix initialized to zero

  // set position covariance (assuming 1 cm position noise)
  double position_noise =
      0.01; // Example position noise in meters (standard deviation)
  covariance[0] = pos_var_;  // x-x covariance
  covariance[7] = pos_var_;  // y-y covariance
  covariance[14] = pos_var_; // z-z covariance

  // set orientation covariance (radian noise)
  covariance[21] = att_var_; // qx-qx covariance
  covariance[28] = att_var_; // qy-qy covariance
  covariance[35] = att_var_; // qz-qz covariance

  std::copy(covariance.begin(), covariance.end(),
            transformed_pose_cov.pose.covariance.begin());

  // publish the transformed pose
  mavros_pub_->publish(transformed_pose_cov);
}
} // namespace mocap_to_vision_pose
