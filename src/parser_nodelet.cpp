#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "data_parser.h"
#include "gnss/parser.h"

namespace gnss_driver {

  class ParserNodelet : public nodelet::Nodelet {
  public:
    ParserNodelet();
    ~ParserNodelet();

  private:
    virtual void onInit();
    
    std::unique_ptr<DataParser> data_parser_;
  };

  void ParserNodelet::onInit() {
    ros::NodeHandle& nh = getPrivateNodeHandle();
    
    std::string gnss_conf;
    std::string raw_data_topic;
    std::string gpgga_topic;
    std::string corr_imu_topic;
    std::string odometry_topic;
    std::string gnss_status_topic;
    std::string ins_status_topic;

    nh.param("gnss_conf", gnss_conf, std::string("./conf/gnss_conf.txt"));
    nh.param("raw_data_topic", raw_data_topic, std::string("/gnss_driver/raw_data"));
    nh.param("gpgga_topic", gpgga_topic, std::string("/gnss_driver/gpgga"));
    nh.param("corr_imu_topic", corr_imu_topic, std::string("/gnss_driver/corrected_imu"));
    nh.param("odometry_topic", odometry_topic, std::string("/gnss_driver/odometry"));
    nh.param("gnss_status_topic", gnss_status_topic, std::string("/gnss_driver/gnss_status"));
    nh.param("ins_status_topic", ins_status_topic, std::string("/gnss_driver/ins_status"));

    data_parser_.reset(new DataParser(nh, raw_data_topic, gpgga_topic, corr_imu_topic,
				      odometry_topic, gnss_status_topic, ins_status_topic));

    if(!data_parser_->init(gnss_conf)) {
      ROS_ERROR("Init parser nodelet failed.");
      ROS_ERROR_STREAM("Init parser nodelet failed.");
      return;
    }

    ROS_INFO("Init parser nodelet success.");    
  }  
}

PLUGINLIB_DECLARE_CLASS(gnss_driver, ParserNodelet,
			gnss_driver::ParserNodelet, nodelet::Nodelet);
