#include <cmath>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "proto/gnss_status.pb.h"

void ins_status_callback(const gnss_driver::InsStatus &ins_status) {
  switch (ins_status.type()) {
  case gnss_driver::InsStatus::GOOD:
    fprintf(stdout, "INS status is GOOD.\r\n");
    break;
  case gnss_driver::InsStatus::CONVERGING:
    fprintf(stdout, "INS status is CONVERGING.\r\n");
    break;
  case gnss_driver::InsStatus::INVALID:
  default:    
    fprintf(stdout, "INS status is INVALID.\r\n");
    break;
  }
}

void stream_status_callback(const gnss_driver::StreamStatus &stream_status) {
  switch (stream_status.ins_stream_type()) {
  case gnss_driver::StreamStatus::CONNECTED:
    fprintf(stdout, "INS stream is CONNECTED.\r\n");
    break;
  case gnss_driver::StreamStatus::DISCONNECTED:
    fprintf(stdout, "INS stream is DISCONNECTED.\r\n");
    break;
  }

  switch (stream_status.rtk_stream_in_type()) {
  case gnss_driver::StreamStatus::CONNECTED:
    fprintf(stdout, "rtk stream in is CONNECTED.\r\n");
    break;
  case gnss_driver::StreamStatus::DISCONNECTED:
    fprintf(stdout, "rtk stream in is DISCONNECTED.\r\n");
    break;
  }
  
  switch (stream_status.rtk_stream_out_type()) {
  case gnss_driver::StreamStatus::CONNECTED:
    fprintf(stdout, "rtk stream out CONNECTED.\r\n");
    break;
  case gnss_driver::StreamStatus::DISCONNECTED:
    fprintf(stdout, "rtk stream out DISCONNECTED.\r\n");
    break;
  }
}

void gnss_status_callback(const gnss_driver::GnssStatus &gnss_status) {
  std::cout << "GNSS status: " << gnss_status.DebugString() << std::endl;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, std::string("gnss_monitor_test"));
  
  ros::NodeHandle nh;
  ros::Subscriber ins_status_publisher;
  ros::Subscriber gnss_status_publisher;
  ros::Subscriber stream_status_publisher;
  
  ins_status_publisher = nh.subscribe("/gnss_driver/ins_status", 16, ins_status_callback);
  gnss_status_publisher = nh.subscribe("/gnss_driver/gnss_status", 16, gnss_status_callback);
  stream_status_publisher = nh.subscribe("/gnss_driver/stream_status", 16, stream_status_callback);
  
  ros::spin();
  
  ROS_ERROR("Exit");
  return 0;
}
