/******************************************************************************
 * Original work Copyright 2017 The Apollo Authors. All Rights
 * Reserved.  
 * 
 * Modified work Copyright 2017 Board of Regents of the
 * Nevada System of Higher Education, on behalf of the University of
 * Nevada, Reno. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <cmath>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "proto/gnss_status.pb.h"

#include "gnss_driver/GnssStatus.h"
#include "gnss_driver/InsStatus.h"
#include "gnss_driver/StreamStatus.h"

void ins_status_callback(const gnss_driver::InsStatus &ins_status) {
  switch (ins_status.type) {
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
  switch (stream_status.ins_stream_type) {
  case gnss_driver::pb::StreamStatus::CONNECTED:
    fprintf(stdout, "INS stream is CONNECTED.\r\n");
    break;
  case gnss_driver::pb::StreamStatus::DISCONNECTED:
    fprintf(stdout, "INS stream is DISCONNECTED.\r\n");
    break;
  }

  switch (stream_status.rtk_stream_in_type) {
  case gnss_driver::pb::StreamStatus::CONNECTED:
    fprintf(stdout, "rtk stream in is CONNECTED.\r\n");
    break;
  case gnss_driver::pb::StreamStatus::DISCONNECTED:
    fprintf(stdout, "rtk stream in is DISCONNECTED.\r\n");
    break;
  }
  
  switch (stream_status.rtk_stream_out_type) {
  case gnss_driver::pb::StreamStatus::CONNECTED:
    fprintf(stdout, "rtk stream out CONNECTED.\r\n");
    break;
  case gnss_driver::pb::StreamStatus::DISCONNECTED:
    fprintf(stdout, "rtk stream out DISCONNECTED.\r\n");
    break;
  }
}

void gnss_status_callback(const gnss_driver::GnssStatus &gnss_status) {
  //std::cout << "GNSS status: " << gnss_status.DebugString() << std::endl;
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
