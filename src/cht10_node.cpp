/**
 * @file /Cht10_serial_func/src/Cht10_serial_func.cpp
 *
 * @brief Implementation for dirver with read data from Cht10 nodelet
 *
 * @author Carl
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <ecl/threads/thread.hpp>
#include <sensor_msgs/LaserScan.h>  
#include <sensor_msgs/Range.h>  
#include <pluginlib/class_list_macros.h>
#include <cht10_node/seiral_func.hpp>


namespace cht10_seiral_func{

class Cht10Func : public nodelet::Nodelet{
#define BUFSIZE 17
#define SCALE 1000

public:
  Cht10Func() : shutdown_requested_(false),serialNumber_("/dev/USB0"),frame_id("laser"){}

  ~Cht10Func(){
    NODELET_DEBUG_STREAM("Waiting for update thread to finish.");
    shutdown_requested_ = true;
    update_thread_.join();
  }

  virtual void onInit(){

    ros::NodeHandle nh = this->getPrivateNodeHandle();
    std::string name = nh.getUnresolvedNamespace();
    nh.getParam("serialNumber", serialNumber_);
    nh.getParam("baudRate", baudRate_);
    nh.getParam("frame_id", frame_id);
    rcv_cnt = 0;
    success_flag = 0;

    fd = open(serialNumber_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
    if(fd < 0){
      ROS_ERROR_STREAM("Open Serial: "<<serialNumber_.c_str()<<" Error!");
      exit(0);
    }

    int countSeq=0;
    scan_pub = nh.advertise<sensor_msgs::Range>("range",100);
    memset(buf, 0, sizeof(buf));
    memset(temp_buf, 0, sizeof(temp_buf));
    memset(result_buf, 0, sizeof(result_buf));
    Cht10driver_.UART0_Init(fd,baudRate_,0,8,1,'N');
    ROS_INFO_STREAM("Open serial: ["<< serialNumber_.c_str() <<" ] successful, with idex: "<<fd<<".");
    NODELET_INFO_STREAM("Cht10Func initialised. Spinning up update thread ... [" << name << "]");
    update_thread_.start(&Cht10Func::update, *this);
  }

  double data_to_meters(int &data, int scale){
    return (double)data/scale;
  }

  void publish_scan(ros::Publisher *pub,
                    double nodes, ros::Time start,
                    std::string frame_id){

    float final_range;
    sensor_msgs::Range range_msg;
    range_msg.field_of_view = 0.05235988;
    range_msg.max_range = 10.0;
    range_msg.min_range = 0.05;
    range_msg.header.frame_id = frame_id;
    range_msg.radiation_type = sensor_msgs::Range::INFRARED;
    if(nodes > range_msg.max_range){
      final_range = std::numeric_limits<float>::infinity();
    }else if(nodes < range_msg.min_range){
      final_range = -std::numeric_limits<float>::infinity();
    }else{
      final_range = nodes;
    }
    range_msg.header.stamp = start;
    range_msg.header.seq = countSeq;
    range_msg.range = final_range;
    scan_pub.publish(range_msg);

  }

  bool get_scan_data(){
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
  }
private:
  void update(){
    rcv_cnt = 0;
    success_flag = 0;
    laser_data = 0;
    ros::Rate spin_rate(50);
    memset(buf, 0, sizeof(buf));
    memset(temp_buf, 0, sizeof(temp_buf));
    memset(result_buf, 0, sizeof(result_buf));
    ROS_INFO_STREAM("Begin receive data from "<<serialNumber_.c_str()<<", with idex:"<<fd<<".");
    fd = open(serialNumber_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
    Cht10driver_.UART0_Init(fd,baudRate_,0,8,1,'N');
    while (! shutdown_requested_ && ros::ok())
    {
      start_scan_time = ros::Time::now();
      success_flag = get_scan_data();
          
      //Send data
      publish_scan(&scan_pub, data_to_meters(laser_data,SCALE),
                       start_scan_time, frame_id);
      spin_rate.sleep();

      countSeq++;
    }

    ROS_INFO_STREAM("Shotdown and close serial: "<<serialNumber_.c_str()<<".");
    Cht10driver_.UART0_Close(fd);
  }
private:
  int fd, len, rcv_cnt;
  int success_flag;
  char buf[40], temp_buf[BUFSIZE],result_buf[BUFSIZE];
  
  Cht10Driver Cht10driver_;
  ecl::Thread update_thread_;
  bool shutdown_requested_;
  ros::Publisher scan_pub;
  int laser_data;
  char data_buf[4];
  // ROS Parameters
  std::string serialNumber_;
  int baudRate_;
  int countSeq;
  
  std::string frame_id;

  ros::Time start_scan_time;
};

} //namespace Cht10_serial_func
PLUGINLIB_EXPORT_CLASS(cht10_seiral_func::Cht10Func,
                        nodelet::Nodelet);
