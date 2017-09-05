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
