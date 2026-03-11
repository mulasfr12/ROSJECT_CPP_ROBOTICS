// includes
// ros2 rclcpp includes
#include "rclcpp/executors.hpp"                         // executors
#include "rclcpp/executors/multi_threaded_executor.hpp" // multithreaded executor
#include "rclcpp/node.hpp"                              // base node
// header includes
#include "cpp_basics_project/robot_interface_class.hpp" // robot interface class
// standard cpp includes
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// instantiate robot interface program as a global variable
std::shared_ptr<RobotInterface> robot_interface;

//~//~// start your function definitions after this line //~//~//
void sleep_ms(int milliseconds);

std::pair<double, double> get_current_linear_and_angular_velocities() {
  return {robot_interface->linear_velocity, robot_interface->angular_velocity};
}

void stop_robot() {
  robot_interface->linear_velocity = 0.0;
  robot_interface->angular_velocity = 0.0;
}

void move_robot_front(double speed) {
  if (speed <= 0.0) {
    stop_robot();
    return;
  }

  robot_interface->linear_velocity = speed;
  robot_interface->angular_velocity = 0.0;
}

void move_robot_back(double speed) {
  if (speed <= 0.0) {
    stop_robot();
    return;
  }

  robot_interface->linear_velocity = -speed;
  robot_interface->angular_velocity = 0.0;
}

void turn_robot_left(double speed) {
  if (speed <= 0.0) {
    stop_robot();
    return;
  }

  robot_interface->linear_velocity = 0.0;
  robot_interface->angular_velocity = speed;
}

void turn_robot_right(double speed) {
  if (speed <= 0.0) {
    stop_robot();
    return;
  }
  robot_interface->angular_velocity = -speed;
  robot_interface->linear_velocity = 0.0;
}

void timed_move_front(int seconds, double speed) {
  if (speed <= 0 || seconds <= 0) {
    stop_robot();
    return;
  }

  move_robot_front(speed);
  sleep_ms(seconds * 1000);
  stop_robot();
}

void timed_move_back(int seconds, double speed) {
  if (seconds <= 0 || speed <= 0.0) {
    stop_robot();
    return;
  }
  move_robot_back(speed);
  sleep_ms(seconds * 1000);
  stop_robot();
}

void timed_turn_left(int seconds, double speed) {
  if (seconds <= 0 || speed <= 0.0) {
    stop_robot();
    return;
  }
  turn_robot_left(speed);
  sleep_ms(seconds * 1000);
  stop_robot();
}

void timed_turn_right(int seconds, double speed) {
  if (seconds <= 0 || speed <= 0.0) {
    stop_robot();
    return;
  }
  turn_robot_right(speed);
  sleep_ms(seconds * 1000);
  stop_robot();
}

void move_distance_front(double dist, double speed) {
  if (dist <= 0 || speed <= 0) {
    stop_robot();
    return;
  }

  double time_s = dist / speed;
  int time_ms = static_cast<int>(time_s * 1000.0);

  move_robot_front(speed);
  sleep_ms(time_ms);
  stop_robot();
}

void move_distance_back(double dist, double speed) {
  if (dist <= 0 || speed <= 0) {
    stop_robot();
    return;
  }

  double time_s = dist / speed;
  int time_ms = static_cast<int>(time_s * 1000.0);

  move_robot_back(speed);
  sleep_ms(time_ms);
  stop_robot();
}

void turn_angle_left(double ang_speed, double angle) {
  if (ang_speed <= 0 || angle <= 0) {
    stop_robot();
    return;
  }

  double time_s = angle / ang_speed;
  int time_ms = static_cast<int>(time_s * 1000);

  turn_robot_left(ang_speed);
  sleep_ms(time_ms);
  stop_robot();
}

void turn_angle_right(double ang_speed, double angle) {
  if (ang_speed <= 0 || angle <= 0) {
    stop_robot();
    return;
  }
  double time_s = angle / ang_speed;
  int time_ms = static_cast<int>(time_s * 1000);

  turn_robot_right(ang_speed);
  sleep_ms(time_ms);
  stop_robot();
}

int angle_to_index(double angle_rad) {
  if (robot_interface->scan_ranges.empty())
    return -1;
  if (robot_interface->scan_angle_increment == 0.0)
    return -1;

  double idx_f = (angle_rad - robot_interface->scan_angle_min) /
                 robot_interface->scan_angle_increment;

  int idx = static_cast<int>(std::round(idx_f));

  if (idx < 0 || idx >= static_cast<int>(robot_interface->scan_ranges.size()))
    return -1;

  return idx;
}

double get_min_scan_angle() { return robot_interface->scan_angle_min; }

double get_max_scan_angle() { return robot_interface->scan_angle_max; }

double get_angle_increment() { return robot_interface->scan_angle_increment; }

double get_min_scan_range() { return robot_interface->scan_range_min; }

double get_max_scan_range() { return robot_interface->scan_range_max; }

std::vector<double> get_all_scan_ranges() {
  return robot_interface->scan_ranges;
}

double get_scan_range_by_index(int index) {
  if (index < 0 ||
      index >= static_cast<int>(robot_interface->scan_ranges.size()))
    return std::numeric_limits<double>::infinity();

  return robot_interface->scan_ranges[index];
}

double get_front_scan_range() {
  int idx = angle_to_index(0.0);
  return (idx == -1) ? std::numeric_limits<double>::infinity()
                     : robot_interface->scan_ranges[idx];
}

double get_left_scan_range() {
  int idx = angle_to_index(robot_interface->pi / 2.0);
  if (idx == -1) {
    return std::numeric_limits<double>::infinity();
  }
  return robot_interface->scan_ranges[idx];
}

double get_right_scan_range() {
  int idx = angle_to_index(-robot_interface->pi / 2.0);
  if (idx == -1) {
    return std::numeric_limits<double>::infinity();
  }
  return robot_interface->scan_ranges[idx];
}

double get_back_scan_range() {
  int idx = angle_to_index(robot_interface->pi);
  if (idx != -1)
    return robot_interface->scan_ranges[idx];

  idx = angle_to_index(-robot_interface->pi);
  return (idx == -1) ? std::numeric_limits<double>::infinity()
                     : robot_interface->scan_ranges[idx];
}
//smallest valid dist,and its idx 
std::pair<double, int> get_min_range_not_inf_with_index() {
  double best = std::numeric_limits<double>::infinity();
  int best_idx = -1;

  for (int i = 0; i < static_cast<int>(robot_interface->scan_ranges.size());
       i++) {
    double value = robot_interface->scan_ranges[i];

    if (std::isinf(value) || std::isnan(value))
      continue;

    if (value < best) {
      best = value;
      best_idx = i;
    }
  }

  return {best, best_idx};
}

//max valid dist,and its idx 
std::pair<double, int> get_max_range_not_inf_with_index() {
  double best = -std::numeric_limits<double>::infinity();
  int best_idx = -1;

  for (int i = 0; i < static_cast<int>(robot_interface->scan_ranges.size());
       i++) {
    double value = robot_interface->scan_ranges[i];

    if (std::isinf(value) || std::isnan(value))
      continue;

    if (value > best) {
      best = value;
      best_idx = i;
    }
  }

  if (best_idx == -1)
    return {std::numeric_limits<double>::infinity(), -1};

  return {best, best_idx};
}

std::map<std::string, double> get_current_position_xyz() {
  std::map<std::string, double> pos;
  pos["x"] = robot_interface->odom_position_x;
  pos["y"] = robot_interface->odom_position_y;
  pos["z"] = robot_interface->odom_position_z;

  return pos;
}

std::map<std::string, double> get_current_orientation_rpy() {
  std::map<std::string, double> ori;
  ori["r"] = robot_interface->odom_orientation_r;
  ori["p"] = robot_interface->odom_orientation_p;
  ori["y"] = robot_interface->odom_orientation_y;

  return ori;
}

//diff in btwn 2 dist   
double get_distance_euclidean_xy(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

void test_all_functions() {
  std::cout << "\n===== TESTING NON-OBJECT-ORIENTED FUNCTIONS =====\n";

  // wait for scan data to arrive
  while (rclcpp::ok() && robot_interface->scan_ranges.empty()) {
    std::cout << "Waiting for scan data...\n";
    sleep_ms(200);
  }

  //  current velocities
  auto vel = get_current_linear_and_angular_velocities();
  std::cout << "[Velocities] linear=" << vel.first << ", angular=" << vel.second
            << "\n";

  std::cout << "[Stop Robot]\n";
  stop_robot();
  sleep_ms(500);

  std::cout << "\n===== PATH STYLE MOVEMENT TEST =====\n";

  std::cout << "Move forward\n";
  timed_move_front(10, 0.10);
  sleep_ms(500);

  std::cout << "Turn right\n";
  timed_turn_right(3, 0.60);
  sleep_ms(500);

  std::cout << "Move forward\n";
  timed_move_front(10, 0.10);
  sleep_ms(500);

  std::cout << "Turn right\n";
  timed_turn_right(3, 0.60);
  sleep_ms(500);

  std::cout << "Move forward\n";
  timed_move_front(4, 0.10);
  sleep_ms(500);

  std::cout << "Turn left\n";
  timed_turn_left(9, 0.60);
  sleep_ms(500);

  std::cout << "Move forward\n";
  timed_move_front(5, 0.10);
  sleep_ms(500);

  std::cout << "Final stop after path test\n";
  stop_robot();
  sleep_ms(500);

  std::cout << "\n===== LASER SCANNER =====\n";
  std::cout << "min_angle = " << get_min_scan_angle() << "\n";
  std::cout << "max_angle = " << get_max_scan_angle() << "\n";
  std::cout << "angle_increment = " << get_angle_increment() << "\n";
  std::cout << "min_range = " << get_min_scan_range() << "\n";
  std::cout << "max_range = " << get_max_scan_range() << "\n";

  auto ranges = get_all_scan_ranges();
  std::cout << "ranges_count = " << ranges.size() << "\n";

  std::cout << "front = " << get_front_scan_range() << "\n";
  std::cout << "back = " << get_back_scan_range() << "\n";
  std::cout << "left = " << get_left_scan_range() << "\n";
  std::cout << "right = " << get_right_scan_range() << "\n";

  auto min_pair = get_min_range_not_inf_with_index();
  std::cout << "min(!inf) = " << min_pair.first << " at index "
            << min_pair.second << "\n";

  auto max_pair = get_max_range_not_inf_with_index();
  std::cout << "max(!inf) = " << max_pair.first << " at index "
            << max_pair.second << "\n";

  std::cout << "\n===== ODOMETRY =====\n";
  auto pos = get_current_position_xyz();
  auto ori = get_current_orientation_rpy();

  std::cout << "pos: x=" << pos["x"] << ", y=" << pos["y"] << ", z=" << pos["z"]
            << "\n";

  std::cout << "ori: r=" << ori["r"] << ", p=" << ori["p"] << ", y=" << ori["y"]
            << "\n";

  std::cout << "distance_xy between (0,0) and (3,4) = "
            << get_distance_euclidean_xy(0, 0, 3, 4) << " (expected 5)\n";

  std::cout << "\n[Final Stop]\n";
  stop_robot();
  sleep_ms(500);

  std::cout << "===== TEST COMPLETE =====\n\n";
}

void sample_move(double linear, double angular) {
  // sample function to move the robot
  // this function can be deleted later!
  // access global variable robot_interface
  robot_interface->linear_velocity = linear;
  robot_interface->angular_velocity = angular;
}

//~//~// finish your function definitions before this line //~//~//

void sleep_ms(int milliseconds) {
  // function to sleep the main thread for specified milliseconds
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void spin_node(rclcpp::executors::MultiThreadedExecutor *executor) {
  // make the robot interface program to run in a separate thread
  // NOTE: THE ROBOT WILL NOT WORK IF THIS FUNCTION IS REMOVED !!!
  executor->spin();
}

int main(int argc, char **argv) {

  // initialize ros2 with cpp
  rclcpp::init(argc, argv);
  // instantiate robot interface program module as a global variable
  // std::shared_ptr<RobotInterface>
  robot_interface = std::make_shared<RobotInterface>();
  // start robot interface program execution
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_interface);
  // run robot interface program in a separate thread
  std::thread executor_thread(spin_node, &executor);
  // wait for a few seconds for program to initialize
  std::cout << "Getting Ready in 5 Seconds... \n";
  sleep_ms(5000);
  std::cout << "READY !!! \n";

  // set global default decimal precision to 3
  std::cout << std::fixed << std::setprecision(3);

  try {
    //~//~// start your program after this line //~//~//
    test_all_functions();
    //~//~// write code here to run only once //~//~//
    sleep_ms(1000);

    //~//~// write code here to run continuously //~//~//

    //~//~// finish your program before this line //~//~//
  } catch (...) {
    // stop the robot
    robot_interface->linear_velocity = 0.000;
    robot_interface->angular_velocity = 0.000;
    sleep_ms(500);
  }
  stop_robot();
  sleep_ms(300);

  executor.cancel();

  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  // shutdown ros2
  executor.remove_node(robot_interface);
  robot_interface.reset();

  rclcpp::shutdown();

  return 0;
}

// End of Code
