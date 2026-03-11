// includes
#include "cpp_basics_project/robot_interface_class.hpp"

int main(int argc, char **argv) {

  // initialize ROS2 node
  rclcpp::init(argc, argv);

  // create an instance of the robot interface class
  std::shared_ptr<RobotInterface> robot_interface =
      std::make_shared<RobotInterface>();

  // create a multithreaded executor
  rclcpp::executors::MultiThreadedExecutor executor;

  // add the robot interface node to the executor
  executor.add_node(robot_interface);

  // spin the executor to handle callbacks
  executor.spin();

  // shutdown ROS2 node when spin completes
  rclcpp::shutdown();

  return 0;
}

// End of Code
