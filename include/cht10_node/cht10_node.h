/**
 * @file /cht10_node/include/cht10_node/cht10_node.h
 *
 * @brief Implementation for dirver with read data from Cht10 nodelet
 *
 * @author Carl
 *
 **/
#ifndef CHT10_NODE_H
#define CHT10_NODE_H
/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <cht10_node/serial_func.hpp>

namespace cht10_serial_func{

class Cht10Func {
#define BUFSIZE 17
#define SCALE 1000

public:
  explicit Cht10Func(rclcpp::Node::SharedPtr & node);

  ~Cht10Func();

  double data_to_meters(unsigned int &data, int scale);

  void publish_scan(double nodes, builtin_interfaces::msg::Time start, std::string frame_id);

  bool get_scan_data();

private:
  void update();

private:
  int fd, len, rcv_cnt;
  int success_flag;
  char buf[40], temp_buf[BUFSIZE],result_buf[BUFSIZE];
  
  Cht10Driver Cht10driver_;
  unsigned int laser_data;
  char data_buf[4];

  //ROS2 
  rclcpp::Node::SharedPtr node_;

  //ROS2 Message
  std::shared_ptr<sensor_msgs::msg::Range> msg_;

  //ROS Topic
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ROS Parameters
  //rclcpp::ParameterService::SharedPtr parameter_service_;
  std::string serialNumber_;
  int baudRate_;
  int countSeq;
  
  std::string frame_id;

  builtin_interfaces::msg::Time start_scan_time;
};

} //namespace cht10_serial_func
#endif //CHT10_NODE_H
