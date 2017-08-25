#ifndef DRIVERS_GNSS_DATA_PARSER_H
#define DRIVERS_GNSS_DATA_PARSER_H

#include <memory>

#include <proj_api.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "gnss/parser.h"

#include "proto/config.pb.h"

// TODO replace these with messages
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
    
    const ros::Subscriber raw_data_sub_;
    const ros::Publisher gpgga_publisher_;
    const ros::Publisher imu_publisher_;
    const ros::Publisher nav_odometry_publisher_;
    const ros::Publisher gnss_status_publisher_;
    const ros::Publisher ins_status_publisher_;
    
    boost::shared_ptr<gnss_driver::pb::GnssStatus> gnss_status_;
    boost::shared_ptr<gnss_driver::pb::InsStatus> ins_status_;
    uint32_t ins_status_record_ = static_cast<uint32_t>(0);
    projPJ wgs84pj_source_;
    projPJ utm_target_;
  };
  
}  // namespace gnss_driver

#endif  // DRIVERS_GNSS_DATA_PARSER_H
