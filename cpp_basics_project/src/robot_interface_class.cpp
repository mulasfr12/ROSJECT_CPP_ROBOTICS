// includes
#include "cpp_basics_project/robot_interface_class.hpp"
#include <cmath>

// class constructor
RobotInterface::RobotInterface() : Node("robot_interface") {
  RCLCPP_INFO(this->get_logger(), "Initializing Robot Interface ...");

  // check if simulation or real-robot
  // checks if /gazebo node is present in list of ros2 nodes
  // gazebo cannot be running along with real robot
  // so if gazebo is running, then it is definitely simulation
  // if not running simulation and not connected to real robot,
  // the result outputs false, meaning real robot - avoided this state!
  char buffer[512]; // Buffer to store output chunks
  std::string result = "";
  FILE *pipe = popen("ros2 node list", "r");
  while (!pipe) {
    // wait for pipe to open - do nothing!
  }
  try {
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
      result += buffer;
    }
  } catch (...) {
    pclose(pipe);
    throw;
  }
  pclose(pipe);
  std::string segment;
  std::vector<std::string> segments;
  std::stringstream ss(result);
  while (std::getline(ss, segment, '\n')) {
    segments.push_back(segment);
  }
  for (const std::string &segment : segments) {
    std::cout << segment << std::endl;
    if (segment == "/gazebo") {
      is_sim = true;
      env_type = "Simulation";
      break;
    } else {
      is_sim = false;
      env_type = "Real-Robot";
    }
  }
  RCLCPP_INFO(this->get_logger(), "Environment: %s", env_type.c_str());

  // declare and initialize cmd_vel publisher
  cmd_vel_pub = this->create_publisher<Twist>("/cmd_vel", 10);
  RCLCPP_INFO(this->get_logger(), "Initialized Publisher: /cmd_vel");

  // declare and initialize callback group
  callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // declare and initialize scan subscriber
  rclcpp::QoS scan_sub_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  scan_sub_options.callback_group = callback_group;
  scan_sub = this->create_subscription<LaserScan>(
      "/scan", scan_sub_qos,
      std::bind(&RobotInterface::scan_callback, this, std::placeholders::_1),
      scan_sub_options);
  RCLCPP_INFO(this->get_logger(), "Initialized Subscriber: /scan");

  // declare and initialize odom subscriber
  rclcpp::QoS odom_sub_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  odom_sub_options.callback_group = callback_group;
  odom_sub = this->create_subscription<Odometry>(
      "/odom", odom_sub_qos,
      std::bind(&RobotInterface::odom_callback, this, std::placeholders::_1),
      odom_sub_options);
  RCLCPP_INFO(this->get_logger(), "Initialized Subscriber: /odom");

  // fastbot does not have an imu
  // declare and initialize imu subscriber
  // rclcpp::QoS imu_sub_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  // imu_sub_options.callback_group = callback_group;
  // imu_sub = this->create_subscription<Imu>(
  //     "/imu", imu_sub_qos,
  //     std::bind(&RobotInterface::imu_callback, this, std::placeholders::_1),
  //     imu_sub_options);
  // RCLCPP_INFO(this->get_logger(), "Initialized Subscriber: /imu");

  // declare and initialize control timer callback
  control_timer = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RobotInterface::control_callback, this), callback_group);
  RCLCPP_INFO(this->get_logger(), "Initialized Control Timer");

  // set default speed limits for the robots
  if (is_sim) {
    // simulation - turtlebot3
    lin_vel_min = 0.000;
    lin_vel_max = 0.150;
    ang_vel_min = 0.000;
    ang_vel_max = 0.450;
  } else {
    // real-robot - fastbot
    lin_vel_min = 0.050;
    lin_vel_max = 0.250;
    ang_vel_min = 0.500;
    ang_vel_max = 1.500;
  }

  RCLCPP_INFO(this->get_logger(), "Robot Interface Initialized !");
}

// class destructor
RobotInterface::~RobotInterface() {
  // indicate robot interface node termination
  RCLCPP_INFO(this->get_logger(), "Terminating Robot Interface ...");
  RCLCPP_INFO(this->get_logger(), "Robot Interface Terminated !");
}

// class methods and callbacks

void RobotInterface::scan_callback(const LaserScan::SharedPtr scan_msg) {
  // simple method to get scan data
  scan_angle_min = scan_msg->angle_min;
  scan_angle_max = scan_msg->angle_max;
  scan_angle_increment = scan_msg->angle_increment;
  scan_range_min = scan_msg->range_min;
  scan_range_max = scan_msg->range_max;
  scan_ranges.clear();
  scan_ranges.assign(scan_msg->ranges.begin(), scan_msg->ranges.end());
}

void RobotInterface::odom_callback(const Odometry::SharedPtr odom_msg) {
  // simple method to get odom data
  odom_position_x = odom_msg->pose.pose.position.x;
  odom_position_y = odom_msg->pose.pose.position.y;
  odom_position_z = odom_msg->pose.pose.position.z;
  angles = euler_from_quaternion(
      odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
  odom_orientation_r = angles["r"];
  odom_orientation_p = angles["p"];
  odom_orientation_y = angles["y"];
}

// fastbot does not have an imu
// void RobotInterface::imu_callback(const Imu::SharedPtr imu_msg) {
//   // simple method to get imu data
//   imu_angular_velocity_x = imu_msg->angular_velocity.x;
//   imu_angular_velocity_y = imu_msg->angular_velocity.y;
//   imu_angular_velocity_z = imu_msg->angular_velocity.z;
//   imu_linear_acceleration_x = imu_msg->linear_acceleration.x;
//   imu_linear_acceleration_y = imu_msg->linear_acceleration.y;
//   imu_linear_acceleration_z = imu_msg->linear_acceleration.z;
// }

void RobotInterface::control_callback() {
  // set robot speeds
  twist_cmd.linear.x = linear_velocity;
  twist_cmd.angular.z = angular_velocity;
  // publish the twist command
  publish_twist_cmd();
  // print debug
  // RCLCPP_INFO(
  //     this->get_logger(),
  //     "lin_vel: %+0.3f, ang_vel: %+0.3f, scan_angle_min: %+0.3f, "
  //     "scan_angle_max: %+0.3f, scan_angle_inc: %+0.3f, scan_range_min: "
  //     "%+0.3f, scan_range_max: %+0.3f, odom_px: %+0.3f, odom_py: %+0.3f, "
  //     "odom_pz: %+0.3f, odom_or: %+0.3f, odom_op: %+0.3f, odom_oy: %+0.3f, "
  //     "imu_ang_vel_x: %+0.3f, imu_ang_vel_y: %+0.3f, imu_ang_vel_z: %+0.3f, "
  //     "imu_lin_acc_x: %+0.3f, imu_lin_acc_y: %+0.3f, imu_lin_acc_z: %+0.3f",
  //     twist_cmd.linear.x, twist_cmd.angular.y, scan_angle_min,
  //     scan_angle_max, scan_angle_increment, scan_range_min, scan_range_max,
  //     odom_position_x, odom_position_y, odom_position_z, odom_orientation_r,
  //     odom_orientation_p, odom_orientation_y, imu_angular_velocity_x,
  //     imu_angular_velocity_y, imu_angular_velocity_z,
  //     imu_linear_acceleration_x, imu_linear_acceleration_y,
  //     imu_linear_acceleration_z);
}

void RobotInterface::publish_twist_cmd() {
  // linear speed control
  double direction_linear = ((twist_cmd.linear.x < 0.0) ? -1.0 : 1.0);
  double velocity_linear = std::fabs(twist_cmd.linear.x);
  if (velocity_linear < lin_vel_min)
    velocity_linear = 0.000;
  if (velocity_linear > lin_vel_max)
    velocity_linear = lin_vel_max;
  twist_cmd.linear.x = (velocity_linear * direction_linear);
  // angular speed control
  double direction_angular = ((twist_cmd.angular.z < 0.0) ? -1.0 : 1.0);
  double velocity_angular = std::fabs(twist_cmd.angular.z);
  if (velocity_angular < ang_vel_min)
    velocity_angular = 0.000;
  if (velocity_angular > ang_vel_max)
    velocity_angular = ang_vel_max;
  twist_cmd.angular.z = (velocity_angular * direction_angular);
  // publish command
  cmd_vel_pub->publish(twist_cmd);
}

std::map<std::string, double>
RobotInterface::euler_from_quaternion(double quat_x, double quat_y,
                                      double quat_z, double quat_w) {
  // function to convert quaternions to euler angles
  // calculate roll
  double sinr_cosp = 2 * (quat_w * quat_x + quat_y * quat_z);
  double cosr_cosp = 1 - 2 * (quat_x * quat_x + quat_y * quat_y);
  double roll = atan2(sinr_cosp, cosr_cosp);
  // calculate pitch
  double sinp = 2 * (quat_w * quat_y - quat_z * quat_x);
  double pitch = asin(sinp);
  // calculate yaw
  double siny_cosp = 2 * (quat_w * quat_z + quat_x * quat_y);
  double cosy_cosp = 1 - 2 * (quat_y * quat_y + quat_z * quat_z);
  double yaw = atan2(siny_cosp, cosy_cosp);
  // store the angle values in a map
  std::map<std::string, double> angles;
  angles["r"] = roll;
  angles["p"] = pitch;
  angles["y"] = yaw;
  // return the angle values
  return angles;
}

// End of Code
