#include <string>
#include <cht10_node/serial_func.hpp>
#include <cstdlib>

#include <iostream>
#include <stdint.h>
#define BUFSIZE 17

int main(int argc, char** argv){
  cht10_serial_func::Cht10Driver cht10driver_;

  std::string serialNumber_;
  serialNumber_ = "/dev/ttyUSB0";
  int baudRate_ = 115200;

  std::stringstream ostream;
  int fd, len, rcv_cnt;
  bool success_flag;
  char buf[40], temp_buf[BUFSIZE],result_buf[BUFSIZE];
  unsigned int laser_data=0;
  char data_buf[4];
  rcv_cnt = 0;
  success_flag = false;
  memset(buf, 0xba, sizeof(buf));
  memset(temp_buf, 0xba, sizeof(temp_buf));
  memset(result_buf, 0xba, sizeof(result_buf));

  fd = open(serialNumber_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
  if(fd < 0){
    std::cout<<"Open Serial: "<<serialNumber_.c_str()<<" Error!";
    exit(0);
  }
  
  cht10driver_.UART0_Init(fd,baudRate_,0,8,1,'N');
  while(1){
    len = cht10driver_.UART0_Recv(fd, buf,40);
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
//          std::cout<<"Received data, with :[";
//          for(int j=0;j<BUFSIZE;j++){
//            printf("%c ",(unsigned char) result_buf[j]);
//          }
//          printf("] \n");
          success_flag = false;

          for(int count = 0; count < 4; count++){
            data_buf[count] = result_buf[9+count];
          }
          sscanf(data_buf, "%x", &laser_data);
          std::cout<<"sensor data:"<<laser_data<<", Distance: "<<(double)laser_data/1000<<std::endl;
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

  
}
