/**
 * @file /cht10_node/src/cht10/cht10_node.cpp
 *
 * @brief Implementation for dirver with read data from Cht10 nodelet
 *
 * @author Carl
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include <functional>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <cht10_node/cht10_node.h>

#define ROS_ERROR RCUTILS_LOG_ERROR
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_ERROR_THROTTLE(sec, ...) RCUTILS_LOG_ERROR_THROTTLE(RCUTILS_STEADY_TIME, sec, __VA_ARGS__)

using namespace cht10_serial_func;

  Cht10Func::Cht10Func(rclcpp::Node::SharedPtr & node):node_(node), 
serialNumber_("/dev/USB0"),
frame_id("laser"){

    msg_ = std::make_shared<sensor_msgs::msg::Range>();

    scan_pub_ = node_->create_publisher<sensor_msgs::msg::Range>("range");

    parameter_service_ = std::make_shared<rclcpp::ParameterService>(node_);

    node_->get_parameter("serialNumber", serialNumber_);
    node_->get_parameter("baudRate", baudRate_);
    node_->get_parameter("frame_id", frame_id);

    rcv_cnt = 0;
    success_flag = 0;

    fd = open(serialNumber_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
    if(fd < 0){
      ROS_ERROR("Open Serial: %s Error!",serialNumber_.c_str());
      exit(0);
    }

    memset(buf, 0, sizeof(buf));
    memset(temp_buf, 0, sizeof(temp_buf));
    memset(result_buf, 0, sizeof(result_buf));
    Cht10driver_.UART0_Init(fd,baudRate_,0,8,1,'N');
    ROS_INFO("Open serial: [ %s ], successfully, with idex: %d.", serialNumber_.c_str() ,fd);

    update();
  }

  Cht10Func::~Cht10Func(){
  }

  double Cht10Func::data_to_meters(unsigned int &data, int scale){
    return (double)data/scale;
  }

  void Cht10Func::publish_scan(double nodes, builtin_interfaces::msg::Time start,
                    std::string frame_id){

    float final_range;
    std::shared_ptr<sensor_msgs::msg::Range> range_msg;
    range_msg->field_of_view = 0.05235988;
    range_msg->max_range = 10.0;
    range_msg->min_range = 0.05;
    range_msg->header.frame_id = frame_id;
    range_msg->radiation_type = sensor_msgs::msg::Range::INFRARED;
    if(nodes > range_msg->max_range){
      final_range = std::numeric_limits<float>::infinity();
    }else if(nodes < range_msg->min_range){
      final_range = -std::numeric_limits<float>::infinity();
    }else{
      final_range = nodes;
    }
    range_msg->header.stamp = start;
    range_msg->range = final_range;

    scan_pub_->publish(range_msg);

  }

  bool Cht10Func::get_scan_data(){
    len = Cht10driver_.UART0_Recv(fd, buf,40);
    if(len>0){
      for(int i = 0; i < len; i++){
        if(rcv_cnt<=BUFSIZE){  
          result_buf[rcv_cnt++] = buf[i];
          if(rcv_cnt == BUFSIZE){
            success_flag = true;
          }
        }//end if
        else{
          /****
          *  checkout received data
          */
          success_flag = false;
           for(int count = 0; count < 4; count++){
            data_buf[count] = result_buf[9+count];
          }
          sscanf(data_buf, "%x", &laser_data);
          
          //std::cout<<"sensor data:"<<laser_data<<std::endl;

          /****
           *  data writing end
           */
          if('~' == buf[i]){
            rcv_cnt = 0;
            result_buf[rcv_cnt++] = buf[i];
          }
        }//end else
      }//end for    
    }
    return success_flag;
  }

  void Cht10Func::update(){
    rcv_cnt = 0;
    success_flag = 0;
    laser_data = 0;
    memset(buf, 0, sizeof(buf));
    memset(temp_buf, 0, sizeof(temp_buf));
    memset(result_buf, 0, sizeof(result_buf));
    ROS_INFO("Begin receive data from %s, with idex: %d.",serialNumber_.c_str(),fd);
    fd = open(serialNumber_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
    Cht10driver_.UART0_Init(fd,baudRate_,0,8,1,'N');
    // Create a function for when messages are to be sent.
    auto publish_message =
      [this]() -> void
      {
//        start_scan_time = 
        rclcpp::Time(start_scan_time);
        success_flag = get_scan_data();
          
        //Send data
        publish_scan(data_to_meters(laser_data,SCALE),
                         start_scan_time, frame_id);
        countSeq++;
      };

    ROS_INFO("Shotdown and close serial: %s.", serialNumber_.c_str());
    Cht10driver_.UART0_Close(fd);

    // Use a timer to schedule periodic message publishing.
    timer_ = node_->create_wall_timer(300ms, publish_message);
  }
