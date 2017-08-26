#include <memory>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "gnss/stream.h"
#include "gnss/utils.h"
#include "proto/config.pb.h"
#include "proto/gnss_status.pb.h"
#include "raw_stream.h"

// ROS msgs
#include "gnss_driver/GnssStatus.h"
#include "gnss_driver/StreamStatus.h"

namespace {
  void switch_stream_status(const gnss_driver::Stream::Status &status,
			    gnss_driver::pb::StreamStatus_Type &report_status_type) {
    switch (status) {
    case gnss_driver::Stream::Status::CONNECTED:
      report_status_type = gnss_driver::pb::StreamStatus::CONNECTED;
      break;

    case gnss_driver::Stream::Status::DISCONNECTED:
      report_status_type = gnss_driver::pb::StreamStatus::DISCONNECTED;
      break;

    case gnss_driver::Stream::Status::ERROR:
    default:
      report_status_type = gnss_driver::pb::StreamStatus::DISCONNECTED;
      break;
    }
  }
}

namespace gnss_driver {
  
  Stream *create_stream(const gnss_driver::pb::Stream &sd) {
    switch (sd.type_case()) {
    case pb::Stream::kSerial:
      if (!sd.serial().has_device()) {
        ROS_ERROR("Serial def has no device field.");
        return nullptr;
      }
      if (!sd.serial().has_baud_rate()) {
        ROS_ERROR_STREAM("Serial def has no baud_rate field. Use default baud rate "
			 << sd.serial().baud_rate());
        return nullptr;
      }
      return Stream::create_serial(sd.serial().device().c_str(),
                                   sd.serial().baud_rate());
      
    case pb::Stream::kTcp:
      if (!sd.tcp().has_address()) {
        ROS_ERROR("tcp def has no address field.");
        return nullptr;
      }
      if (!sd.tcp().has_port()) {
        ROS_ERROR("tcp def has no port field.");
        return nullptr;
      }
      return Stream::create_tcp(sd.tcp().address().c_str(), sd.tcp().port());
      
    case pb::Stream::kUdp:
      if (!sd.udp().has_address()) {
        ROS_ERROR("tcp def has no address field.");
        return nullptr;
      }
      if (!sd.udp().has_port()) {
        ROS_ERROR("tcp def has no port field.");
        return nullptr;
      }
      return Stream::create_udp(sd.udp().address().c_str(), sd.udp().port());
      
    case pb::Stream::kNtrip:
      if (!sd.ntrip().has_address()) {
        ROS_ERROR("ntrip def has no address field.");
        return nullptr;
      }
      if (!sd.ntrip().has_port()) {
        ROS_ERROR("ntrip def has no port field.");
        return nullptr;
      }
      if (!sd.ntrip().has_mount_point()) {
        ROS_ERROR("ntrip def has no mount point field.");
        return nullptr;
      }
      if (!sd.ntrip().has_user()) {
        ROS_ERROR("ntrip def has no user field.");
        return nullptr;
      }
      if (!sd.ntrip().has_password()) {
        ROS_ERROR("ntrip def has no passwd field.");
        return nullptr;
      }
      return Stream::create_ntrip(sd.ntrip().address(), sd.ntrip().port(), sd.ntrip().mount_point(),
				  sd.ntrip().user(), sd.ntrip().password(), sd.ntrip().timeout_s());
    default:
      return nullptr;
    }
  }
  
  RawStream::RawStream(ros::NodeHandle &nh, const std::string &name,
		       const std::string &raw_data_topic,
		       const std::string &rtcm_data_topic,
		       const std::string &stream_status_topic)
    : raw_data_topic_(raw_data_topic),
      rtcm_data_topic_(rtcm_data_topic),
      raw_data_publisher_(nh.advertise<std_msgs::String>(raw_data_topic_, 256)),
      rtcm_data_publisher_(nh.advertise<std_msgs::String>(rtcm_data_topic_, 256)),
      stream_status_publisher_(nh.advertise<gnss_driver::StreamStatus>(stream_status_topic, 256,
								       true)) {
    stream_status_.reset(new gnss_driver::pb::StreamStatus());
  }

  RawStream::~RawStream() {
    this->logout();
    this->disconnect();
  }
  
  bool RawStream::init(const std::string &cfg_file) {
    if (!stream_status_) {
      ROS_ERROR_STREAM("New stream status failed.");
      return false;
    }
    stream_status_->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    stream_status_->set_ins_stream_type(gnss_driver::pb::StreamStatus::DISCONNECTED);
    stream_status_->set_rtk_stream_in_type(gnss_driver::pb::StreamStatus::DISCONNECTED);
    stream_status_->set_rtk_stream_out_type(gnss_driver::pb::StreamStatus::DISCONNECTED);

    gnss_driver::StreamStatus ros_stream_status;
    ros_stream_status.header.stamp = ros::Time::now();
    ros_stream_status.ins_stream_type = gnss_driver::StreamStatus::DISCONNECTED;
    ros_stream_status.rtk_stream_in_type = gnss_driver::StreamStatus::DISCONNECTED;
    ros_stream_status.rtk_stream_out_type = gnss_driver::StreamStatus::DISCONNECTED;
    
    stream_status_publisher_.publish(ros_stream_status);

    if (!parse_config_text(cfg_file, &config_)) {
      ROS_INFO("Parse config context failed.");
      return false;
    }
    ROS_INFO_STREAM("Loaded config:\n" << config_.DebugString());
    
    // Creates streams.
    Stream *s = nullptr;
    if (!config_.has_data()) {
      ROS_INFO("Error: Config file must provide the data stream.");
      return false;
    }
    s = create_stream(config_.data());
    if (s == nullptr) {
      ROS_ERROR("Failed to create data stream.");
      return false;
    }
    data_stream_.reset(s);
    
    Status *status = new Status();
    if (!status) {
      ROS_ERROR("Failed to create data stream status.");
      return false;
    }
    data_stream_status_.reset(status);
    
    if (config_.has_command()) {
      s = create_stream(config_.command());
      if (s == nullptr) {
	ROS_ERROR("Failed to create command stream.");
	return false;
      }
      command_stream_.reset(s);
      
      status = new Status();
      if (!status) {
	ROS_ERROR("Failed to create command stream status.");
	return false;
      }
      command_stream_status_.reset(status);
    } else {
      command_stream_ = data_stream_;
      command_stream_status_ = data_stream_status_;
    }
    
    if (config_.has_rtk_from()) {
      s = create_stream(config_.rtk_from());
      if (s == nullptr) {
	ROS_ERROR("Failed to create rtk_from stream.");
	return false;
      }
      in_rtk_stream_.reset(s);
      
      status = new Status();
      if (!status) {
	ROS_ERROR("Failed to create rtk_from stream status.");
	return false;
      }
      in_rtk_stream_status_.reset(status);
      
      if (config_.has_rtk_to()) {
	s = create_stream(config_.rtk_to());
	if (s == nullptr) {
	  ROS_ERROR("Failed to create rtk_to stream.");
	  return false;
	}
	out_rtk_stream_.reset(s);
	
	status = new Status();
	if (!status) {
	  ROS_ERROR("Failed to create rtk_to stream status.");
	  return false;
	}
	out_rtk_stream_status_.reset(status);
      } else {
	out_rtk_stream_ = data_stream_;
	out_rtk_stream_status_ = data_stream_status_;
      }
      
      if (config_.has_rtk_solution_type()) {
	if (config_.rtk_solution_type() ==
	    gnss_driver::pb::Config::RTK_SOFTWARE_SOLUTION) {
	  rtk_software_solution_ = true;
	}
      }
    }
    
    if (config_.login_commands_size() == 0) {
      ROS_WARN("No login_commands in config file.");
    }
    
    if (config_.logout_commands_size() == 0) {
      ROS_WARN("No logout_commands in config file.");
    }
    
    // connect and login
    if (!connect()) {
      ROS_ERROR("gnss driver connect failed.");
      return false;
    }
    
    if (!login()) {
      ROS_ERROR("gnss driver login failed.");
      return false;
    }
    
    data_thread_ptr_.reset(new std::thread(&RawStream::data_spin, this));
    ntrip_thread_ptr_.reset(new std::thread(&RawStream::ntrip_spin, this));
    
    return true;
  }

  bool RawStream::connect() {
    if (data_stream_) {
      if (data_stream_->get_status() != Stream::Status::CONNECTED) {
	if (!data_stream_->connect()) {
	  ROS_ERROR("data stream connect failed.");
	  return false;
	}
	data_stream_status_->status = Stream::Status::CONNECTED;
	stream_status_->set_ins_stream_type(gnss_driver::pb::StreamStatus::CONNECTED);
      }
    }
    
    if (command_stream_) {
      if (command_stream_->get_status() != Stream::Status::CONNECTED) {
	if (!data_stream_->connect()) {
	  ROS_ERROR("command stream connect failed.");
	  return false;
	}
	command_stream_status_->status = Stream::Status::CONNECTED;
      }
    }
    
    if (in_rtk_stream_) {
      if (in_rtk_stream_->get_status() != Stream::Status::CONNECTED) {
	if (!in_rtk_stream_->connect()) {
	  ROS_ERROR("in rtk stream connect failed.");
	} else {
	  in_rtk_stream_status_->status = Stream::Status::CONNECTED;
	  stream_status_->set_rtk_stream_in_type(gnss_driver::pb::StreamStatus::CONNECTED);
	}
      }
    } else {
      stream_status_->set_rtk_stream_in_type(gnss_driver::pb::StreamStatus::CONNECTED);
    }
    
    if (out_rtk_stream_) {
      if (out_rtk_stream_->get_status() != Stream::Status::CONNECTED) {
	if (!out_rtk_stream_->connect()) {
	  ROS_ERROR("out rtk stream connect failed.");
	} else {
	  out_rtk_stream_status_->status = Stream::Status::CONNECTED;
	  stream_status_->set_rtk_stream_out_type(gnss_driver::pb::StreamStatus::CONNECTED);
	}
      }
    } else {
      stream_status_->set_rtk_stream_out_type(gnss_driver::pb::StreamStatus::CONNECTED);
    }
    return true;
  }
  
  
  bool RawStream::disconnect() {
    if (data_stream_) {
      if (data_stream_->get_status() == Stream::Status::CONNECTED) {
	if (!data_stream_->disconnect()) {
	  ROS_ERROR("data stream disconnect failed.");
	  return false;
	}
      }
    }
    
    if (command_stream_) {
      if (command_stream_->get_status() == Stream::Status::CONNECTED) {
	if (!data_stream_->disconnect()) {
	  ROS_ERROR("command stream disconnect failed.");
	  return false;
	}
      }
    }
    if (in_rtk_stream_) {
      if (in_rtk_stream_->get_status() == Stream::Status::CONNECTED) {
	if (!in_rtk_stream_->disconnect()) {
	  ROS_ERROR("in rtk stream disconnect failed.");
	  return false;
	}
      }
    }
    if (out_rtk_stream_) {
      if (out_rtk_stream_->get_status() == Stream::Status::CONNECTED) {
	if (!out_rtk_stream_->disconnect()) {
	  ROS_ERROR("out rtk stream disconnect failed.");
	  return false;
	}
      }
    }
    
    return true;
  }

  bool RawStream::login() {
    std::vector<std::string> login_data;
    for (const auto& login_command : config_.login_commands()) {
      command_stream_->write(login_command);
      login_data.emplace_back(login_command);
      ROS_INFO_STREAM("Login command: " << login_command);
      // sleep a little to avoid overun of the slow serial interface.
      ros::Duration(0.2).sleep();
    }
    command_stream_->register_login_data(login_data);
    return true;
  }
  
  bool RawStream::logout() {
    for (const auto& logout_command : config_.logout_commands()) {
      command_stream_->write(logout_command);
      ROS_INFO_STREAM("Logout command: " << logout_command);
    }
    return true;
  }

  void RawStream::stream_status_check() {
    bool status_report = false;
    gnss_driver::pb::StreamStatus_Type report_stream_status;
    
    if (data_stream_ &&
	(data_stream_->get_status() != data_stream_status_->status)) {
      data_stream_status_->status = data_stream_->get_status();
      status_report = true;
      switch_stream_status(data_stream_status_->status, report_stream_status);
      stream_status_->set_ins_stream_type(report_stream_status);
    }
    
    if (in_rtk_stream_ &&
	(in_rtk_stream_->get_status() != in_rtk_stream_status_->status)) {
      in_rtk_stream_status_->status = in_rtk_stream_->get_status();
      status_report = true;
      switch_stream_status(in_rtk_stream_status_->status, report_stream_status);
      stream_status_->set_rtk_stream_in_type(report_stream_status);
    }
    
    if (out_rtk_stream_ &&
	(out_rtk_stream_->get_status() != out_rtk_stream_status_->status)) {
      out_rtk_stream_status_->status = out_rtk_stream_->get_status();
      status_report = true;
      switch_stream_status(out_rtk_stream_status_->status, report_stream_status);
      stream_status_->set_rtk_stream_out_type(report_stream_status);
    }
    
    if (status_report) {
      stream_status_->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());

      gnss_driver::StreamStatus ros_stream_status;
      ros_stream_status.header.stamp = ros::Time::now();

      ros_stream_status.ins_stream_type = stream_status_->ins_stream_type();
      ros_stream_status.rtk_stream_in_type = stream_status_->rtk_stream_in_type();
      ros_stream_status.rtk_stream_out_type = stream_status_->rtk_stream_out_type();      
      stream_status_publisher_.publish(ros_stream_status);
    }
  }


  void RawStream::data_spin() {
    stream_status_->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());

    gnss_driver::StreamStatus ros_stream_status;
    ros_stream_status.header.stamp = ros::Time::now();
    
    ros_stream_status.ins_stream_type = stream_status_->ins_stream_type();
    ros_stream_status.rtk_stream_in_type = stream_status_->rtk_stream_in_type();
    ros_stream_status.rtk_stream_out_type = stream_status_->rtk_stream_out_type();      
    stream_status_publisher_.publish(ros_stream_status);
    
    while (ros::ok()) {
      size_t length = data_stream_->read(buffer_, BUFFER_SIZE);
      if (length > 0) {
	std_msgs::StringPtr msg_pub(new std_msgs::String);
	if (!msg_pub) {
	  ROS_ERROR("New data sting msg failed.");
	  continue;
	}
	msg_pub->data.assign(reinterpret_cast<const char *>(buffer_), length);
	raw_data_publisher_.publish(msg_pub);
      }
      stream_status_check();
    }
  }


  void RawStream::ntrip_spin() {
    if (in_rtk_stream_ == nullptr) {
      return;
    }
    while (ros::ok()) {
      size_t length = in_rtk_stream_->read(buffer_ntrip_, BUFFER_SIZE);
      if (length > 0) {
	if (rtk_software_solution_) {
	  std_msgs::StringPtr rtkmsg_pub(new std_msgs::String);
	  if (!rtkmsg_pub) {
	    ROS_ERROR("New rtkmsg failed.");
	    continue;
	  }
	  rtkmsg_pub->data.assign(reinterpret_cast<const char *>(buffer_ntrip_),
				  length);
	  rtcm_data_publisher_.publish(rtkmsg_pub);
	} else {
	  if (out_rtk_stream_ == nullptr) {
	    continue;
	  }
	  size_t ret = out_rtk_stream_->write(buffer_ntrip_, length);
	  if (ret != length) {
	    ROS_ERROR_STREAM("Expect write out rtk stream bytes "
			     << length << " but got " << ret);
	  }
	}
      }
    }
  }
    
} // namespace gnss_driver
