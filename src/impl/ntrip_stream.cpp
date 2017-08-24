#include <unistd.h>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "gnss/stream.h"
#include "gnss/utils.h"
#include "tcp_stream.h"

#include "proto/gpgga.pb.h"

namespace {
  
  template <typename T>
  constexpr bool is_zero(T value) {
    return value == static_cast<T>(0);
  }
}

namespace gnss_driver {

  class NtripStream : public TcpStream {
  public:
    NtripStream(const std::string& address, uint16_t port,
		const std::string& mountpoint, const std::string& user,
		const std::string& passwd, uint32_t timeout_s);
    ~NtripStream();
    
    virtual size_t read(uint8_t* buffer, size_t max_length);
    virtual bool connect();
    virtual bool disconnect();
    
  private:
    void reconnect();
    bool is_login_ = false;
    const std::string mountpoint_;
    const std::string login_data_;
    const std::string gga_data_;
    double timeout_s_ = 60.0;
    double data_active_s_ = 0.0;
    ros::Subscriber gpgga_data_sub_;
    std::string gpgga_data_topic_;
    void gpgga_data_callback(const std_msgs::String::ConstPtr& msg);
  };

  NtripStream::NtripStream(const std::string& address, uint16_t port,
			   const std::string& mountpoint, const std::string& user,
			   const std::string& passwd, uint32_t timeout_s)
    : TcpStream(address.c_str(), port, 0),
      mountpoint_(mountpoint),
      // add dasha gpgga at first
      login_data_("GET /" + mountpoint +
                  " HTTP/1.0\r\n"
                  "User-Agent: NTRIP gnss_driver/0.0\r\n"
                  "accept: */* \r\n"
                  "Authorization: Basic " +
                  encode_base64(user + ":" + passwd) + "\r\n\r\n"),
      gga_data_("GET /" + mountpoint +
                " HTTP/1.0\r\n"
                "User-Agent: NTRIP gnss_driver/0.0\r\n"
                "accept: */* \r\n\r\n"),
      timeout_s_(timeout_s) {
    ros::NodeHandle nh;
    nh.param("gpgga_data_topic", _gpgga_data_topic,
	     std::string("/apollo/sensor/gnss/gpgga"));
    gpgga_data_sub_ = nh.subscribe(gpgga_data_topic_, 1000,
				   &NtripStream::gpgga_data_callback, this);
  }
  
  NtripStream::~NtripStream() { this->disconnect(); }

  bool NtripStream::connect() {
    if (is_login_) {
      return true;
    }
    if (!TcpStream::connect()) {
      ROS_ERROR("Tcp connect failed.");
      return false;
    }
    
    uint8_t buffer[2048];
    size_t size = 0;
    size_t try_times = 0;
    
    size = write(reinterpret_cast<const uint8_t*>(login_data_.data()),
		 login_data_.size());
    if (size != login_data_.size()) {
      TcpStream::disconnect();
      status_ = Stream::Status::ERROR;
      ROS_ERROR("Send ntrip request failed.");
      return false;
    }

    bzero(buffer, sizeof(buffer));
    ROS_INFO("Read ntrip response.");
    size = TcpStream::read(buffer, 2048);
    if (is_login_) {
      ROS_INFO("Already login.");
      return true;
    }
    while ((size == 0) && (try_times < 3)) {
      sleep(1);
      size = TcpStream::read(buffer, 2048);
      ++try_times;
      if (is_login_) {
	ROS_INFO("Already login.");
	return true;
      }
    }
    
    if (!size) {
      TcpStream::disconnect();
      status_ = Stream::Status::DISCONNECTED;
      ROS_ERROR("No response from ntripcaster.");
      return false;
    }

    if (std::strstr(reinterpret_cast<char*>(buffer), "ICY 200 OK\r\n")) {
      status_ = Stream::Status::CONNECTED;
      is_login_ = true;
      return true;
    }
    
    if (std::strstr(reinterpret_cast<char*>(buffer), "SOURCETABLE 200 OK\r\n")) {
      ROS_ERROR_STREAM("Mountpoint " << mountpoint_ << " not exist.");
    }

    if (std::strstr(reinterpret_cast<char*>(buffer), "HTTP/")) {
      ROS_ERROR("Authentication failed.");
    }

    ROS_INFO_STREAM("No expect data.");
    ROS_INFO_STREAM("Recv data length: " << size);
    // ROS_INFO_STREAM("Data from server: " << reinterpret_cast<char*>(buffer));

    TcpStream::disconnect();
    status_ = Stream::Status::ERROR;
    return false;
  }
  
  bool NtripStream::disconnect() {
    if (is_login_) {
      bool ret = TcpStream::disconnect();
      if (!ret) {
	return false;
      }
      is_login_ = false;
    }
    
    return true;
  }
  
  void NtripStream::reconnect() {
    ROS_INFO("Reconnect ntrip caster.");
    disconnect();
    connect();
    if (status_ != Stream::Status::CONNECTED) {
      ROS_INFO("Reconnect ntrip caster failed.");
      return;
    }
    
    data_active_s_ = ros::Time::now().toSec();
    ROS_INFO("Reconnect ntrip caster success.");
  }
  
  size_t NtripStream::read(uint8_t* buffer, size_t max_length) {
    size_t ret = 0;
    
    if (status_ != Stream::Status::CONNECTED) {
      reconnect();
      if (status_ != Stream::Status::CONNECTED) {
	return 0;
      }
    }
    
    if (is_zero(data_active_s_)) {
      data_active_s_ = ros::Time::now().toSec();
    }
    
    ret = TcpStream::read(buffer, max_length);
    if (ret) {
      data_active_s_ = ros::Time::now().toSec();
    }
    
    // timeout detect
    if ((ros::Time::now().toSec() - data_active_s_) > timeout_s_) {
      ROS_INFO("Ntrip timeout.");
      reconnect();
    }
    
    return ret;
  }

  void NtripStream::gpgga_data_callback(const std_msgs::String::ConstPtr& msg) {
    size_t size_gga = 0;
    apollo::drivers::gnss::gpgga::GPGGA gpgga;
    gpgga.ParseFromString(msg->data);
    std::string gga_request = _gga_data + gpgga.gpgga();
    size_gga = write(reinterpret_cast<const uint8_t*>(gga_request.data()),
		     gga_request.size());
    if (size_gga != gga_request.size()) {
      TcpStream::disconnect();
      status_ = Stream::Status::ERROR;
      ROS_ERROR("Send ntrip gga request failed.");
    }
    
    // TODO:more response check, discard response now
  }

  Stream* Stream::create_ntrip(const std::string& address, uint16_t port,
			       const std::string& mountpoint,
			       const std::string& user, const std::string& passwd,
			       uint32_t timeout_s) {
    return new NtripStream(address, port, mountpoint, user, passwd, timeout_s);
  }
  
}
