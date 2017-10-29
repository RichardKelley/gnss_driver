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

#ifndef DRIVERS_GNSS_DATA_PARSER_H
#define DRIVERS_GNSS_DATA_PARSER_H

#include <memory>

#include <proj_api.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "gnss/parser.h"

#include "proto/config.pb.h"

#include "proto/gnss.pb.h"
#include "proto/imu.pb.h"
#include "proto/ins.pb.h"
#include "proto/gnss_status.pb.h"

// ROS msgs
#include "gnss_driver/Gnss.h"
#include "gnss_driver/Imu.h"
#include "gnss_driver/Ins.h"
#include "gnss_driver/GnssStatus.h"
#include "gnss_driver/InsStatus.h"
#include "gnss_driver/StreamStatus.h"

#include <sensor_msgs/Imu.h>

namespace gnss_driver {

  class DataParser {
  public:
    DataParser(ros::NodeHandle &nh, const std::string &raw_data_topic,
	       const std::string &gpgga_topic, const std::string &corr_imu_topic,
	       const std::string &odometry_topic,
	       const std::string &gnss_status_topic,
	       const std::string &ins_status_topic);
    ~DataParser() {}
    bool init(const std::string &cfg_file);
    
  private:
    void raw_data_callback(const std_msgs::String::ConstPtr &msg);
    void dispatch_message(Parser::MessageType type, MessagePtr message);
    void publish_odometry_pb_message(const MessagePtr message);
    void publish_corrimu_pb_message(const MessagePtr message);
    void check_ins_status(::gnss_driver::pb::Ins *ins);
    void check_gnss_status(::gnss_driver::pb::Gnss *gnss);
    
    bool inited_flag_ = false;
    std::unique_ptr<Parser> data_parser_;
    
    sensor_msgs::Imu ros_imu_;

    const ros::Subscriber raw_data_sub_;
    const ros::Publisher gpgga_publisher_;
    const ros::Publisher imu_publisher_;
    const ros::Publisher nav_odometry_publisher_;
    const ros::Publisher gnss_status_publisher_;
    const ros::Publisher ins_status_publisher_;
    const ros::Publisher ros_imu_publisher_;

    // Publishes IMU data using a sensor_msgs/Imu.
    const ros::Timer ros_imu_timer_;
    void publish_ros_imu(const ros::TimerEvent& e);
    
    boost::shared_ptr<gnss_driver::pb::GnssStatus> gnss_status_;
    boost::shared_ptr<gnss_driver::pb::InsStatus> ins_status_;
    uint32_t ins_status_record_ = static_cast<uint32_t>(0);
    projPJ wgs84pj_source_;
    projPJ utm_target_;
  };
  
}  // namespace gnss_driver

#endif  // DRIVERS_GNSS_DATA_PARSER_H
