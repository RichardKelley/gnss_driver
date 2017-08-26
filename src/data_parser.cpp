#include <proj_api.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <Eigen/Geometry>
#include <boost/array.hpp>
#include <cmath>
#include <memory>

#include "data_parser.h"
#include "gnss/parser.h"
#include "gnss/utils.h"
#include "proto/gnss.pb.h"
#include "proto/gpgga.pb.h"
#include "proto/imu.pb.h"
#include "proto/ins.pb.h"
#include "util/time_conversion.h"

// ROS msgs
#include "gnss_driver/Gnss.h"
#include "gnss_driver/GPGGA.h"
#include "gnss_driver/Imu.h"
#include "gnss_driver/Ins.h"
#include "gnss_driver/Gps.h"

#include "gps.pb.h"
#include "imu.pb.h"

namespace gnss_driver {

  namespace {

    constexpr double DEG_TO_RAD_LOCAL = M_PI / 180.0;
    const std::string WGS84_TEXT = "+proj=latlong +ellps=WGS84";

    // Notice that we use UTM zone 11 for Nevada here. -rck
    const std::string UTM_TARGET =
      "+proj=utm +zone=11 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs ";
    
    // covariance data for pose if can not get from novatel inscov topic
    static const boost::array<double, 36> POSE_COVAR = {
      2, 0, 0, 0,    0, 0, 0, 2, 0, 0, 0,    0, 0, 0, 2, 0, 0, 0,
      0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01};

    template <class T>
    void publish_message_raw(const ros::Publisher &pub, const T *pb) {
      std_msgs::String msg_pub;
      
      if (pb->SerializeToString(&msg_pub.data)) {
	pub.publish(msg_pub);
	return;
      }
      ROS_ERROR("Failed to serialize message.");
    }
    
    Parser *create_parser(pb::Stream::Format format,
			  bool is_base_station = false) {
      switch (format) {
      case pb::Stream::NOVATEL_BINARY:
	return Parser::create_novatel();
	
      default:
	return nullptr;
      }
    }
  } // namespace 
  
  DataParser::DataParser(ros::NodeHandle& nh, const std::string& raw_data_topic,
			 const std::string& gpgga_topic,
			 const std::string& corr_imu_topic,
			 const std::string& odometry_topic,
			 const std::string& gnss_status_topic,
			 const std::string& ins_status_topic)
    : raw_data_sub_(nh.subscribe(raw_data_topic, 256,
                                 &DataParser::raw_data_callback, this)),
      gpgga_publisher_(nh.advertise<std_msgs::String>(gpgga_topic, 64)),
      imu_publisher_(nh.advertise<gnss_driver::Imu>(corr_imu_topic, 64)),
      nav_odometry_publisher_(nh.advertise<gnss_driver::Gps>(odometry_topic, 64)),
      gnss_status_publisher_(nh.advertise<gnss_driver::GnssStatus>(
								       gnss_status_topic, 64, true)),
      ins_status_publisher_(nh.advertise<gnss_driver::InsStatus>(ins_status_topic,
								     64, true)) {
    std::string utm_target_param;
    nh.param("proj4_text", utm_target_param, UTM_TARGET);
    ROS_INFO_STREAM("proj4_text : " << utm_target_param);
    
    wgs84pj_source_ = pj_init_plus(WGS84_TEXT.c_str());
    utm_target_ = pj_init_plus(utm_target_param.c_str());
    gnss_status_.reset(new gnss_driver::pb::GnssStatus());
    ins_status_.reset(new gnss_driver::pb::InsStatus());
    if (gnss_status_) {
      gnss_status_->set_solution_status(0);
      gnss_status_->set_num_sats(0);
      gnss_status_->set_position_type(0);
      gnss_status_->set_solution_completed(false);
    }
    
    if (ins_status_) {
      ins_status_->set_type(gnss_driver::pb::InsStatus::INVALID);
    }
  }

  bool DataParser::init(const std::string &cfg_file) {
    pb::Config config;
    if ((!ins_status_) || (!gnss_status_)) {
      ROS_ERROR_STREAM("New ins status or gnss status failed.");
      return false;
    }
    ins_status_->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    gnss_status_->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    // TODO
    //ins_status_publisher_.publish(ins_status_);
    //gnss_status_publisher_.publish(gnss_status_);
    if (!parse_config_text(cfg_file, &config)) {
      ROS_FATAL_STREAM("Failed to load config file: " << cfg_file);
      return false;
    }
    
    ROS_INFO_STREAM("Creating data parser of format: " << config.data().format());
    data_parser_.reset(create_parser(config.data().format(), false));
    if (!data_parser_) {
      ROS_FATAL("Failed to create data parser.");
      return false;
    }
    
    inited_flag_ = true;
    return true;
  }

  void DataParser::raw_data_callback(const std_msgs::String::ConstPtr &msg) {
    if (!inited_flag_) {
      return;
    }

    data_parser_->update(msg->data);
    Parser::MessageType type;
    MessagePtr msg_ptr;
    
    for (;;) {
      type = data_parser_->get_message(msg_ptr);
      if (type == Parser::MessageType::NONE) break;
      dispatch_message(type, msg_ptr);
    }

  }

  void DataParser::check_ins_status(gnss_driver::pb::Ins *ins) {

    if (ins_status_record_ != static_cast<uint32_t>(ins->type())) {
      ins_status_record_ = static_cast<uint32_t>(ins->type());
      switch (ins->type()) {
      case gnss_driver::pb::Ins::GOOD:
        ins_status_->set_type(gnss_driver::pb::InsStatus::GOOD);
        break;
	
      case gnss_driver::pb::Ins::CONVERGING:
        ins_status_->set_type(gnss_driver::pb::InsStatus::CONVERGING);
        break;
	
      case gnss_driver::pb::Ins::INVALID:
      default:
        ins_status_->set_type(gnss_driver::pb::InsStatus::INVALID);
        break;
      }
      ins_status_->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
      // TODO
      //ins_status_publisher_.publish(ins_status_);
    }

  }

  void DataParser::check_gnss_status(gnss_driver::pb::Gnss *gnss) {
    gnss_status_->set_solution_status(
				      static_cast<uint32_t>(gnss->solution_status()));
    gnss_status_->set_num_sats(static_cast<uint32_t>(gnss->num_sats()));
    gnss_status_->set_position_type(static_cast<uint32_t>(gnss->position_type()));
    
    if (static_cast<uint32_t>(gnss->solution_status()) == 0) {
      gnss_status_->set_solution_completed(true);
    } else {
      gnss_status_->set_solution_completed(false);
    }
    gnss_status_->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    // TODO
    //gnss_status_publisher_.publish(gnss_status_);

  }

  void DataParser::dispatch_message(Parser::MessageType type,
				    MessagePtr message) {
    std_msgs::String msg_pub;
    
    switch (type) {
    case Parser::MessageType::GNSS:
      check_gnss_status(As<gnss_driver::pb::Gnss>(message));
      break;
      
    case Parser::MessageType::INS:
      check_ins_status(As<gnss_driver::pb::Ins>(message));
      publish_corrimu_pb_message(message);
      publish_odometry_pb_message(message);
      break;
      
    case Parser::MessageType::GPGGA:
      publish_message_raw(gpgga_publisher_,
                          As<gnss_driver::pb::GPGGA>(message));
    default:
      break;
    }
  }

  void DataParser::publish_odometry_pb_message(const MessagePtr message) {

    gnss_driver::pb::Ins *ins = As<gnss_driver::pb::Ins>(message);
    boost::shared_ptr<gnss_driver::pb::Gps> gps(new gnss_driver::pb::Gps());
    if (!gps) {
      ROS_ERROR("New gps failed.");
      return;
    }
    
    double unix_sec = gnss_driver::gps2unix(ins->measurement_time());
    gps->mutable_header()->set_timestamp_sec(unix_sec);
    auto *gps_msg = gps->mutable_localization();
    double pub_sec = ros::Time::now().toSec();
    ROS_DEBUG_STREAM("gps timeline odometry delay: " << pub_sec - unix_sec
		     << " s.");
    
    // 1. pose xyz, TODO:
    double x = ins->position().lon();
    ;
    double y = ins->position().lat();
    x *= DEG_TO_RAD_LOCAL;
    y *= DEG_TO_RAD_LOCAL;
    
    int ret = pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);
    if (ret != 0) {
      ROS_ERROR_STREAM("prj transform failed, x: " << x << ", y: " << y
		       << ", ret: " << ret);
      return;
    }
    
    gps_msg->mutable_position()->set_x(x);
    gps_msg->mutable_position()->set_y(y);
    gps_msg->mutable_position()->set_z(ins->position().height());
    
    // 2. orientation, TODO: check
    Eigen::Quaterniond q =
      Eigen::AngleAxisd(ins->euler_angles().z() - 90 * DEG_TO_RAD_LOCAL,
                        Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-ins->euler_angles().y(), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(ins->euler_angles().x(), Eigen::Vector3d::UnitY());
    
    gps_msg->mutable_orientation()->set_qx(q.x());
    gps_msg->mutable_orientation()->set_qy(q.y());
    gps_msg->mutable_orientation()->set_qz(q.z());
    gps_msg->mutable_orientation()->set_qw(q.w());
    
    gps_msg->mutable_linear_velocity()->set_x(ins->linear_velocity().x());
    gps_msg->mutable_linear_velocity()->set_y(ins->linear_velocity().y());
    gps_msg->mutable_linear_velocity()->set_z(ins->linear_velocity().z());
    
    gps_msg->mutable_angular_velocity()->set_x(ins->angular_velocity().x());
    gps_msg->mutable_angular_velocity()->set_y(ins->angular_velocity().y());
    gps_msg->mutable_angular_velocity()->set_z(ins->angular_velocity().z());
    
    /*
    ROS_DEBUG_STREAM("local timeline odometry delay: "
		     << (ros::Time::now().toSec() - ins->mutable_header()->timestamp_sec())
		     << " s.");
    */

    gnss_driver::Gps ros_gps;
    ros_gps.header.stamp = ros::Time::now();
    ros_gps.localization.position.x = gps_msg->position().x();
    ros_gps.localization.position.y = gps_msg->position().y();
    ros_gps.localization.position.z = gps_msg->position().z();

    ros_gps.localization.orientation.qx = gps_msg->orientation().qx();
    ros_gps.localization.orientation.qy = gps_msg->orientation().qy();
    ros_gps.localization.orientation.qz = gps_msg->orientation().qz();
    ros_gps.localization.orientation.qw = gps_msg->orientation().qw();

    ros_gps.localization.linear_velocity.x = gps_msg->linear_velocity().x();
    ros_gps.localization.linear_velocity.y = gps_msg->linear_velocity().y();
    ros_gps.localization.linear_velocity.z = gps_msg->linear_velocity().z();

    ros_gps.localization.angular_velocity.x = gps_msg->angular_velocity().x();
    ros_gps.localization.angular_velocity.y = gps_msg->angular_velocity().y();
    ros_gps.localization.angular_velocity.z = gps_msg->angular_velocity().z();

    // TODO
    nav_odometry_publisher_.publish(ros_gps);

  }
  
  void DataParser::publish_corrimu_pb_message(const MessagePtr message) {
    gnss_driver::pb::Ins *ins = As<gnss_driver::pb::Ins>(message);
    boost::shared_ptr<gnss_driver::pb::Imu> imu(new gnss_driver::pb::Imu());
    if (!imu) {
      ROS_ERROR("New imu failed.");
      return;
    }
    double unix_sec = gnss_driver::gps2unix(ins->measurement_time());
    imu->mutable_header()->set_timestamp_sec(unix_sec);
    double pub_sec = ros::Time::now().toSec();
    ROS_DEBUG_STREAM("gps timeline imu delay: " << pub_sec - unix_sec << " s.");
    
    //auto *imu_msg = imu->mutable_imu();
    //imu_msg->mutable_linear_acceleration()->set_x(-ins->linear_acceleration().y());
    //imu_msg->mutable_linear_acceleration()->set_y(ins->linear_acceleration().x());
    //imu_msg->mutable_linear_acceleration()->set_z(ins->linear_acceleration().z());
    
    //imu_msg->mutable_angular_velocity()->set_x(-ins->angular_velocity().y());
    //imu_msg->mutable_angular_velocity()->set_y(ins->angular_velocity().x());
    //imu_msg->mutable_angular_velocity()->set_z(ins->angular_velocity().z());

    ROS_DEBUG_STREAM("local timeline imu delay: "
		     << (ros::Time::now().toSec() - ins->mutable_header()->timestamp_sec())
		     << " s.");

    // TODO publish
    //imu_publisher_.publish(imu);

  }
  

  
} // namespace gnss_driver
