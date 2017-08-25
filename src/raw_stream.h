#ifndef GNSS_DRIVER_RAW_STREAM_H
#define GNSS_DRIVER_RAW_STREAM_H

#include <memory>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "gnss/stream.h"
#include "proto/config.pb.h"
#include "proto/gnss_status.pb.h"

namespace gnss_driver {

  class RawStream {
  public:
    RawStream(ros::NodeHandle &nh, const std::string &name,
	      const std::string &raw_topic, const std::string &rtcm_topic,
	      const std::string &stream_status_topic);
    ~RawStream();
    bool init(const std::string &cfg_file);
    
    struct Status {
      bool filter[Stream::NUM_STATUS] = {false};
      Stream::Status status;
    };
    
  private:
    void data_spin();
    void ntrip_spin();
    bool connect();
    bool disconnect();
    bool login();
    bool logout();
    void stream_status_check();
    
    static constexpr size_t BUFFER_SIZE = 2048;
    uint8_t buffer_[BUFFER_SIZE];
    uint8_t buffer_ntrip_[BUFFER_SIZE];
    
    std::shared_ptr<Stream> data_stream_;
    std::shared_ptr<Stream> command_stream_;
    std::shared_ptr<Stream> in_rtk_stream_;
    std::shared_ptr<Stream> out_rtk_stream_;
    
    std::shared_ptr<Status> data_stream_status_;
    std::shared_ptr<Status> command_stream_status_;
    std::shared_ptr<Status> in_rtk_stream_status_;
    std::shared_ptr<Status> out_rtk_stream_status_;
    
    bool rtk_software_solution_ = false;
    bool is_healthy_ = true;
    pb::Config config_;
    
    const std::string raw_data_topic_;
    const std::string rtcm_data_topic_;
    const ros::Publisher raw_data_publisher_;
    const ros::Publisher rtcm_data_publisher_;
    const ros::Publisher stream_status_publisher_;
    
    boost::shared_ptr<gnss_driver::pb::StreamStatus> stream_status_;
    std::unique_ptr<std::thread> data_thread_ptr_;
    std::unique_ptr<std::thread> ntrip_thread_ptr_;
  };
  
} // namespace gnss_driver

#endif // GNSS_DRIVER_RAW_STREAM_H
