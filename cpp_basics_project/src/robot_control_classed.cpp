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
#include <vector>

//~//~// start your class after this line //~//~//
void sleep_ms(int milliseconds);

class RobotControl {
public:
  // class constructor
  RobotControl(std::shared_ptr<RobotInterface> robot_interface)
      : robot_interface_(robot_interface) {}

  // class destructor
  ~RobotControl() {}

  std::pair<double, double> get_current_linear_and_angular_velocities() {
    return {robot_interface_->linear_velocity,
            robot_interface_->angular_velocity};
  }

  void stop_robot() {
    robot_interface_->linear_velocity = 0.0;
    robot_interface_->angular_velocity = 0.0;
  }

  void move_robot_front(double speed) {
    if (speed <= 0.0) {
      stop_robot();
      return;
    }
    robot_interface_->linear_velocity = speed;
    robot_interface_->angular_velocity = 0.0;
  }

  void move_robot_back(double speed) {
    if (speed <= 0.0) {
      stop_robot();
      return;
    }
    robot_interface_->linear_velocity = -speed;
    robot_interface_->angular_velocity = 0.0;
  }

  void turn_robot_left(double ang_speed) {
    if (ang_speed <= 0.0) {
      stop_robot();
      return;
    }
    robot_interface_->linear_velocity = 0.0;
    robot_interface_->angular_velocity = ang_speed;
  }

  void turn_robot_right(double ang_speed) {
    if (ang_speed <= 0.0) {
      stop_robot();
      return;
    }
    robot_interface_->linear_velocity = 0.0;
    robot_interface_->angular_velocity = -ang_speed;
  }

  void timed_move_front(int seconds, double speed) {
    if (seconds <= 0 || speed <= 0.0) {
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

  void timed_turn_left(int seconds, double ang_speed) {
    if (seconds <= 0 || ang_speed <= 0.0) {
      stop_robot();
      return;
    }
    turn_robot_left(ang_speed);
    sleep_ms(seconds * 1000);
    stop_robot();
  }

  void timed_turn_right(int seconds, double ang_speed) {
    if (seconds <= 0 || ang_speed <= 0.0) {
      stop_robot();
      return;
    }
    turn_robot_right(ang_speed);
    sleep_ms(seconds * 1000);
    stop_robot();
  }

  void move_distance_front(double dist, double speed) {
    if (dist <= 0.0 || speed <= 0.0) {
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
    if (dist <= 0.0 || speed <= 0.0) {
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
    if (ang_speed <= 0.0 || angle <= 0.0) {
      stop_robot();
      return;
    }
    double time_s = angle / ang_speed;
    int time_ms = static_cast<int>(time_s * 1000.0);

    turn_robot_left(ang_speed);
    sleep_ms(time_ms);
    stop_robot();
  }

  void turn_angle_right(double ang_speed, double angle) {
    if (ang_speed <= 0.0 || angle <= 0.0) {
      stop_robot();
      return;
    }
    double time_s = angle / ang_speed;
    int time_ms = static_cast<int>(time_s * 1000.0);

    turn_robot_right(ang_speed);
    sleep_ms(time_ms);
    stop_robot();
  }

  double get_min_scan_angle() { return robot_interface_->scan_angle_min; }

  double get_max_scan_angle() { return robot_interface_->scan_angle_max; }

  double get_angle_increment() {
    return robot_interface_->scan_angle_increment;
  }

  double get_min_scan_range() { return robot_interface_->scan_range_min; }

  double get_max_scan_range() { return robot_interface_->scan_range_max; }

  std::vector<double> get_all_scan_ranges() {
    return robot_interface_->scan_ranges;
  }

  double get_scan_range_by_index(int index) {
    if (index < 0 ||
        index >= static_cast<int>(robot_interface_->scan_ranges.size())) {
      return std::numeric_limits<double>::infinity();
    }
    return robot_interface_->scan_ranges[index];
  }

  double get_front_scan_range() { return get_scan_range_by_angle(0.0); }

  double get_back_scan_range() {
    double v = get_scan_range_by_angle(robot_interface_->pi);
    if (!std::isinf(v))
      return v;
    return get_scan_range_by_angle(-robot_interface_->pi);
  }

  double get_left_scan_range() {
    return get_scan_range_by_angle(robot_interface_->pi / 2.0);
  }

  double get_right_scan_range() {
    return get_scan_range_by_angle(-robot_interface_->pi / 2.0);
  }

  std::pair<double, int> get_min_range_not_inf_with_index() {
    double best = std::numeric_limits<double>::infinity();
    int best_idx = -1;

    for (int i = 0; i < static_cast<int>(robot_interface_->scan_ranges.size());
         i++) {
      double v = robot_interface_->scan_ranges[i];
      if (std::isinf(v) || std::isnan(v))
        continue;
      if (v < best) {
        best = v;
        best_idx = i;
      }
    }

    return {best, best_idx};
  }

  std::pair<double, int> get_max_range_not_inf_with_index() {
    double best = -std::numeric_limits<double>::infinity();
    int best_idx = -1;

    for (int i = 0; i < static_cast<int>(robot_interface_->scan_ranges.size());
         i++) {
      double v = robot_interface_->scan_ranges[i];
      if (std::isinf(v) || std::isnan(v))
        continue;
      if (v > best) {
        best = v;
        best_idx = i;
      }
    }

    if (best_idx == -1) {
      return {std::numeric_limits<double>::infinity(), -1};
    }

    return {best, best_idx};
  }

  std::map<std::string, double> get_current_position_xyz() {
    return {
        {"x", robot_interface_->odom_position_x},
        {"y", robot_interface_->odom_position_y},
        {"z", robot_interface_->odom_position_z},
    };
  }

  std::map<std::string, double> get_current_orientation_rpy() {
    return {
        {"r", robot_interface_->odom_orientation_r},
        {"p", robot_interface_->odom_orientation_p},
        {"y", robot_interface_->odom_orientation_y},
    };
  }

  double get_distance_euclidean_xy(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
  }

  //   double get_front90_min_distance_m() {
  //     if (robot_interface_->scan_ranges.empty() ||
  //         robot_interface_->scan_angle_increment == 0.0)
  //       return std::numeric_limits<double>::infinity();

  //     int right_idx = angle_to_index(-robot_interface_->pi / 4.0);
  //     int left_idx = angle_to_index(robot_interface_->pi / 4.0);

  //     if (right_idx == -1 || left_idx == -1)
  //       return std::numeric_limits<double>::infinity();

  //     int start = std::min(right_idx, left_idx);
  //     int end = std::max(right_idx, left_idx);

  //     double min_val = std::numeric_limits<double>::infinity();
  //     for (int i = start; i <= end; i++) {
  //       double v = robot_interface_->scan_ranges[i];
  //       if (std::isinf(v) || std::isnan(v))
  //         continue;
  //       if (v < min_val)
  //         min_val = v;
  //     }
  //     return min_val;
  //   }

  void obstacle_prediction(double threshold_m = 0.500) {
    if (robot_interface_->scan_ranges.empty()) {
      std::cout << "[ObstaclePrediction] no scan data yet\n";
      return;
    }
    //get front90 degrees
    int right_idx = angle_to_index(-robot_interface_->pi / 4.0);
    int left_idx = angle_to_index(robot_interface_->pi / 4.0);

    if (right_idx == -1 || left_idx == -1) {
      std::cout << "[ObstaclePrediction] invalid front scan indices\n";
      return;
    }

    int start = std::min(right_idx, left_idx);
    int end = std::max(right_idx, left_idx);

    //loop thru frint90 nd get values
    std::vector<double> front_ranges;
    for (int i = start; i <= end; i++) {
      double value = robot_interface_->scan_ranges[i];
      if (std::isinf(value) || std::isnan(value)) {
        continue;
      }
      front_ranges.push_back(value);
    }

    if (front_ranges.empty()) {
      std::cout << "[ObstaclePrediction] none\n";
      return;
    }
    //get min dist to obj
    double min_val = std::numeric_limits<double>::infinity();
    for (double value : front_ranges) {
      if (value < min_val) {
        min_val = value;
      }
    }

    if (min_val > threshold_m) {
      std::cout << "[ObstaclePrediction] none\n";
      return;
    }

    std::vector<int> front_cm;
    for (double value : front_ranges) {
      front_cm.push_back(static_cast<int>(value * 100.0));
    }
    //count each range val occured
    std::map<int, int> freq;
    for (int cm : front_cm) {
      freq[cm]++;
    }

    //distinct value appearance - check for if wall or obst
    int repeated_ranges_count = 0;
    for (const auto &pair : freq) {
      if (pair.second > 1) {
        repeated_ranges_count++;
      }
    }

    if (repeated_ranges_count > 1) {
      std::cout << "[ObstaclePrediction] obstacle\n";
    } else {
      std::cout << "[ObstaclePrediction] wall\n";
    }
  }

  void direction_tracking() {
    double yaw = robot_interface_->odom_orientation_y;

    const double pi = robot_interface_->pi;
    const double two_pi = 2.0 * pi;
    const double sector = two_pi / 16.0;

    double shifted = yaw + pi;

    //ensuring angle stays witihn valid circ interval
    while (shifted < 0.0) {
      shifted += two_pi;
    }
    while (shifted >= two_pi) {
      shifted -= two_pi;
    }
    //rounding to nearest circ
    shifted += sector / 2.0;
    if (shifted >= two_pi) {
      shifted -= two_pi;
    }
    //angle to size of a sect
    int raw = static_cast<int>(shifted / sector);
    //recentering mapping, ensures 0 = north etc
    int index = raw - 8;
    if (index < 0) {
      index += 16;
    }

    static const std::vector<std::string> directions = {
        "-N-", "NNW", "N-W", "WNW", "-W-", "WSW", "S-W", "SSW",
        "-S-", "SSE", "S-E", "ESE", "-E-", "ENE", "N-E", "NNE"};

    std::cout << "[DirectionTracking] " << directions[index] << "\n";
  }

  double min_in_segment(double center_angle_rad, double half_width_rad) {
    if (robot_interface_->scan_ranges.empty() ||
        robot_interface_->scan_angle_increment == 0.0) {
      return std::numeric_limits<double>::infinity();
    }

    int idx_a = angle_to_index(center_angle_rad - half_width_rad);
    int idx_b = angle_to_index(center_angle_rad + half_width_rad);

    if (idx_a == -1 || idx_b == -1) {
      return std::numeric_limits<double>::infinity();
    }

    int start = std::min(idx_a, idx_b);
    int end = std::max(idx_a, idx_b);

    double best = std::numeric_limits<double>::infinity();

    for (int i = start; i <= end; ++i) {
      double value = robot_interface_->scan_ranges[i];
      if (std::isinf(value) || std::isnan(value)) {
        continue;
      }
      if (value < best) {
        best = value;
      }
    }

    return best;
  }

  struct SegmentMins {
    double left;
    double front_left;
    double front;
    double front_right;
    double right;
  };

  SegmentMins get_five_segment_mins_45deg() {
    const double pi = robot_interface_->pi;
    const double half = pi / 8.0;

    SegmentMins m;
    m.left = min_in_segment(pi / 2.0, half);
    m.front_left = min_in_segment(pi / 4.0, half);
    m.front = min_in_segment(0.0, half);
    m.front_right = min_in_segment(-pi / 4.0, half);
    m.right = min_in_segment(-pi / 2.0, half);

    return m;
  }

  void naive_obstacle_avoider(double threshold_m = 0.350, int max_seconds = 300,
                              double fwd_speed = 0.10, double turn_speed = 0.80,
                              int step_ms = 120, int turn_ms = 320) {
    while (rclcpp::ok() && robot_interface_->scan_ranges.empty()) {
      std::cout << "[Avoider] Waiting for scan data...\n";
      sleep_ms(200);
    }

    auto prev_pos = get_current_position_xyz();
    double prev_x = prev_pos["x"];
    double prev_y = prev_pos["y"];
    double total_dist = 0.0;

    auto start_time = std::chrono::steady_clock::now();
    auto deadline = start_time + std::chrono::seconds(max_seconds);

    std::cout << "\n[Avoider] Started\n";

    int cycle = 1;

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      SegmentMins m = get_five_segment_mins_45deg();

      std::cout << "\n========================================\n";
      std::cout << "[Cycle " << cycle << "]\n";

      if (cycle % 3 == 1) {
        std::cout << "\n--- Obstacle Prediction ---\n";
        obstacle_prediction(threshold_m);

        std::cout << "\n--- Direction Tracking ---\n";
        direction_tracking();
      }

      std::cout << "\n--- Scan Segment Minimums ---\n";
      std::cout << std::fixed << std::setprecision(3);
      std::cout << "[Ranges] "
                << "L=" << m.left << " FL=" << m.front_left << " F=" << m.front
                << " FR=" << m.front_right << " R=" << m.right << "\n";

      std::cout << "\n--- Decision ---\n";

      if (m.front_left > threshold_m && m.front > threshold_m &&
          m.front_right > threshold_m) {
        std::cout << "[Avoider] Moving forward\n";
        move_robot_front(fwd_speed);
        sleep_ms(step_ms);

      } else if (m.front_left < threshold_m && m.front >= threshold_m &&
                 m.front_right >= threshold_m) {
        std::cout << "[Avoider] Turning right\n";
        turn_robot_right(turn_speed);
        sleep_ms(turn_ms);
        stop_robot();
        sleep_ms(80);

        move_robot_front(fwd_speed);
        sleep_ms(step_ms);
      } else if (m.front_right < threshold_m && m.front >= threshold_m &&
                 m.front_left >= threshold_m) {
        std::cout << "[Avoider] Turning left\n";
        turn_robot_left(turn_speed);
        sleep_ms(turn_ms);
        stop_robot();
        sleep_ms(80);

        move_robot_front(fwd_speed);
        sleep_ms(step_ms);
      } else {
        bool go_left = (m.left >= m.right);

        std::cout << "[Avoider] Front blocked, turning "
                  << (go_left ? "left" : "right") << "\n";

        if (go_left) {
          turn_robot_left(turn_speed);
        } else {
          turn_robot_right(turn_speed);
        }

        sleep_ms(turn_ms);
        stop_robot();
        sleep_ms(120);
      }

      auto cur_pos = get_current_position_xyz();
      double cur_x = cur_pos["x"];
      double cur_y = cur_pos["y"];

      double step_dist =
          get_distance_euclidean_xy(prev_x, prev_y, cur_x, cur_y);
      total_dist += step_dist;

      prev_x = cur_x;
      prev_y = cur_y;

      auto v = get_current_linear_and_angular_velocities();

      std::cout << "\n--- Motion Log ---\n";
      std::cout << "[Motion] total_dist=" << total_dist
                << " m, cmd_lin=" << v.first << ", cmd_ang=" << v.second
                << "\n";

      std::cout << "========================================\n";

      cycle++;
    }

    std::cout << "\n[Avoider] Finished\n";
    stop_robot();
    sleep_ms(200);
  }

  void run_algorithm_tasks() {
    std::cout << "\n===== RUNNING ALGORITHM TASKS =====\n";

    while (rclcpp::ok() && get_all_scan_ranges().empty()) {
      std::cout << "Waiting for scan data...\n";
      sleep_ms(200);
    }

    std::cout << "\n--- Naive Obstacle Avoider With Logs ---\n";
    naive_obstacle_avoider(0.350, 300, 0.10, 0.70, 120, 300);

    std::cout << "\n===== ALGORITHM TASKS COMPLETE =====\n";
  }

protected:
  //~//~// add protected functions & variables after this line //~//~//

private:
  //~//~// add private functions & variables after this line //~//~//

  std::shared_ptr<RobotInterface> robot_interface_;

  double get_scan_range_by_angle(double angle_rad) {
    if (robot_interface_->scan_ranges.empty() ||
        robot_interface_->scan_angle_increment == 0.0) {
      return std::numeric_limits<double>::infinity();
    }

    double idx_f = (angle_rad - robot_interface_->scan_angle_min) /
                   robot_interface_->scan_angle_increment;

    int idx = static_cast<int>(std::round(idx_f));

    if (idx < 0 ||
        idx >= static_cast<int>(robot_interface_->scan_ranges.size())) {
      return std::numeric_limits<double>::infinity();
    }

    return robot_interface_->scan_ranges[idx];
  }

  int angle_to_index(double angle_rad) {
    if (robot_interface_->scan_ranges.empty() ||
        robot_interface_->scan_angle_increment == 0.0) {
      return -1;
    }

    double idx_f = (angle_rad - robot_interface_->scan_angle_min) /
                   robot_interface_->scan_angle_increment;

    int idx = static_cast<int>(std::round(idx_f));

    if (idx < 0 ||
        idx >= static_cast<int>(robot_interface_->scan_ranges.size())) {
      return -1;
    }

    return idx;
  }

}; // class RobotControl

//~//~// finish your class before this line //~//~//
void test_all_functions(RobotControl &rc) {
  std::cout << "\n===== TESTING ROBOTCONTROL FUNCTIONS =====\n";

  // wait for scan data to arrive
  while (rclcpp::ok() && rc.get_all_scan_ranges().empty()) {
    std::cout << "Waiting for scan data...\n";
    sleep_ms(200);
  }

  // current velocities
  auto v = rc.get_current_linear_and_angular_velocities();
  std::cout << "[Velocities] linear=" << v.first << ", angular=" << v.second
            << "\n";

  std::cout << "[Stop Robot]\n";
  rc.stop_robot();
  sleep_ms(500);

  std::cout << "\n===== PATH STYLE MOVEMENT TEST =====\n";

  std::cout << "Move forward\n";
  rc.timed_move_front(12, 0.10);
  sleep_ms(500);

  std::cout << "Turn right\n";
  rc.timed_turn_right(5, 0.60);
  sleep_ms(500);

  std::cout << "Move forward\n";
  rc.timed_move_front(15, 0.10);
  sleep_ms(500);

  std::cout << "Turn right\n";
  rc.timed_turn_right(5, 0.60);
  sleep_ms(500);

  std::cout << "Move forward\n";
  rc.timed_move_front(4, 0.10);
  sleep_ms(500);

  std::cout << "Turn left\n";
  rc.timed_turn_left(12, 0.60);
  sleep_ms(500);

  std::cout << "Move forward\n";
  rc.timed_move_front(6, 0.10);
  sleep_ms(500);

  std::cout << "Final stop after path test\n";
  rc.stop_robot();
  sleep_ms(500);

  std::cout << "\n===== LASER SCANNER =====\n";
  std::cout << "min_angle = " << rc.get_min_scan_angle() << "\n";
  std::cout << "max_angle = " << rc.get_max_scan_angle() << "\n";
  std::cout << "angle_increment = " << rc.get_angle_increment() << "\n";
  std::cout << "min_range = " << rc.get_min_scan_range() << "\n";
  std::cout << "max_range = " << rc.get_max_scan_range() << "\n";

  auto ranges = rc.get_all_scan_ranges();
  std::cout << "ranges_count = " << ranges.size() << "\n";

  std::cout << "front = " << rc.get_front_scan_range() << "\n";
  std::cout << "back = " << rc.get_back_scan_range() << "\n";
  std::cout << "left = " << rc.get_left_scan_range() << "\n";
  std::cout << "right = " << rc.get_right_scan_range() << "\n";

  auto min_pair = rc.get_min_range_not_inf_with_index();
  std::cout << "min(!inf) = " << min_pair.first << " at index "
            << min_pair.second << "\n";

  auto max_pair = rc.get_max_range_not_inf_with_index();
  std::cout << "max(!inf) = " << max_pair.first << " at index "
            << max_pair.second << "\n";

  std::cout << "\n===== ODOMETRY =====\n";
  auto pos = rc.get_current_position_xyz();
  auto ori = rc.get_current_orientation_rpy();

  std::cout << "pos: x=" << pos["x"] << ", y=" << pos["y"] << ", z=" << pos["z"]
            << "\n";

  std::cout << "ori: r=" << ori["r"] << ", p=" << ori["p"] << ", y=" << ori["y"]
            << "\n";

  std::cout << "distance_xy between (0,0) and (3,4) = "
            << rc.get_distance_euclidean_xy(0, 0, 3, 4) << " (expected 5)\n";

  std::cout << "\n[Final Stop]\n";

  rc.stop_robot();
  sleep_ms(500);

  std::cout << "===== TEST COMPLETE =====\n\n";
}
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
  std::shared_ptr<RobotInterface> robot_interface =
      std::make_shared<RobotInterface>();
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

    ////////// |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~| //////////
    ////////// | ! YOU ARE ON YOUR OWN FROM HERE ! | //////////
    ////////// |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~| //////////

    //~//~// write code here to run only once //~//~//

    RobotControl rc(robot_interface);
    // rc.naive_obstacle_avoider(0.40, 300, 0.10, 0.60, 100, 1000);
    // test_all_functions(rc);
    rc.run_algorithm_tasks();
    // const double threshold = 0.30;
    // const double fwd_speed = 0.10;
    // const double turn_speed = 0.60;
    // const int step_ms = 200;
    //~//~// write code here to run continuously //~//~//
    // while (true) {

    // rc.direction_tracking();

    //   double dmin = rc.get_front90_min_distance_m();
    //   rc.obstacle_prediction(threshold);

    //   if (dmin > threshold) {

    //     rc.move_robot_front(fwd_speed);
    //     sleep_ms(step_ms);

    //   } else {
    //     rc.stop_robot();
    //     sleep_ms(100);

    //     rc.turn_robot_left(turn_speed);
    //     sleep_ms(400);

    //     rc.stop_robot();
    //     sleep_ms(100);
    //   }
    //}

    //~//~// finish your program before this line //~//~//
  } catch (...) {
    // stop the robot
    robot_interface->linear_velocity = 0.000;
    robot_interface->angular_velocity = 0.000;
    sleep_ms(300);
  }

  // shutdown ros2
  robot_interface->linear_velocity = 0.000;
  robot_interface->angular_velocity = 0.000;
  sleep_ms(300);

  executor.cancel();

  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  executor.remove_node(robot_interface);
  robot_interface.reset();

  rclcpp::shutdown();

  return 0;
}

// End of Code
