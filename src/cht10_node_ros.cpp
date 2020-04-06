#include <cht10_node/cht10_node.h>

int main(int argc, char * argv[])
{
  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Create a node.
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("cht10_node");

  cht10_serial_func::Cht10Func cht(node);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;

}
