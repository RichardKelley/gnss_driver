// An parser for decoding binary messages from a NovAtel receiver. The following
// messages must be
// logged in order for this parser to work properly.
//
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include "gnss/novatel_messages.h"
#include "gnss/parser.h"
#include "proto/gnss.pb.h"
#include "proto/imu.pb.h"
#include "proto/ins.pb.h"

namespace gnss_driver {

  // Anonymous namespace that contains helper constants and functions.
  namespace {
    
    constexpr size_t BUFFER_SIZE = 256;
    constexpr int SECONDS_PER_WEEK = 60 * 60 * 24 * 7;
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();
    
    // The NovAtel's orientation covariance matrix is pitch, roll, and yaw. We use
    // the index array below
    // to convert it to the orientation covariance matrix with order roll, pitch,
    // and yaw.
    constexpr int INDEX[] = {4, 3, 5, 1, 0, 2, 7, 6, 8};
    static_assert(sizeof(INDEX) == 9 * sizeof(int), "Incorrect size of INDEX");
    
    template <typename T>
    constexpr bool is_zero(T value) {
      return value == static_cast<T>(0);
    }
    
    // CRC algorithm from the NovAtel document.
    inline uint32_t crc32_word(uint32_t word) {
      for (int j = 0; j < 8; ++j) {
	if (word & 1) {
	  word = (word >> 1) ^ 0xEDB88320;
	} else {
	  word >>= 1;
	}
      }
      return word;
    }

    inline uint32_t crc32_block(const uint8_t* buffer, size_t length) {
      uint32_t word = 0;
      while (length--) {
	uint32_t t1 = (word >> 8) & 0xFFFFFF;
	uint32_t t2 = crc32_word((word ^ *buffer++) & 0xFF);
	word = t1 ^ t2;
      }
      return word;
    }
    
    // Converts NovAtel's azimuth (north = 0, east = 90) to FLU yaw (east = 0, north
    // = pi/2).
    constexpr double azimuth_deg_to_yaw_rad(double azimuth) {
      return (90.0 - azimuth) * DEG_TO_RAD;
    }
    
    // A helper that fills an Point3D object (which uses the FLU frame) using RFU
    // measurements.
    inline void rfu_to_flu(double r, double f, double u,
			   gnss_driver::pb::Point3D* flu) {
      flu->set_x(f);
      flu->set_y(-r);
      flu->set_z(u);
    }
    
  }  // namespace
  
  class NovatelParser : public Parser {
  public:
    NovatelParser();
    virtual MessageType get_message(MessagePtr& message_ptr);
    
  private:
    bool check_crc();
    Parser::MessageType prepare_message(MessagePtr& message_ptr);
    
    // The handle_xxx functions return whether a message is ready.
    bool handle_best_pos(const novatel::BestPos* pos, uint16_t gps_week,
			 uint32_t gps_millisecs);
    bool handle_best_vel(const novatel::BestVel* vel, uint16_t gps_week,
			 uint32_t gps_millisecs);
    bool handle_corr_imu_data(const novatel::CorrImuData* imu);
    bool handle_ins_cov(const novatel::InsCov* cov);
    bool handle_ins_pva(const novatel::InsPva* pva);
    bool handle_raw_imu_x(const novatel::RawImuX* imu);
    
    double gyro_scale_ = 0.0;
    double accel_scale_ = 0.0;
    float imu_measurement_span_ = 1.0 / 200.0;
    float imu_measurement_hz_ = 200.0;
    
    // TODO: Get mapping from configuration file.
    int imu_frame_mapping_ = 5;
    double imu_measurement_time_previous_ = -1.0;
    std::vector<uint8_t> buffer_;
    size_t header_length_ = 0;
    size_t total_length_ = 0;
    
    // -1 is an unused value.
    novatel::SolutionStatus solution_status_ =
      static_cast<novatel::SolutionStatus>(-1);
    novatel::SolutionType position_type_ = static_cast<novatel::SolutionType>(-1);
    novatel::SolutionType velocity_type_ = static_cast<novatel::SolutionType>(-1);
    novatel::InsStatus ins_status_ = static_cast<novatel::InsStatus>(-1);
    
    gnss_driver::pb::Gnss gnss_;
    gnss_driver::pb::Imu imu_;
    gnss_driver::pb::Ins ins_;
  };
  
  Parser* Parser::create_novatel() { return new NovatelParser(); }
  
  NovatelParser::NovatelParser() {
    buffer_.reserve(BUFFER_SIZE);
    ins_.mutable_position_covariance()->Resize(9, FLOAT_NAN);
    ins_.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
    ins_.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);
  }
  
  Parser::MessageType NovatelParser::get_message(MessagePtr& message_ptr) {
    if (data_ == nullptr) {
      return MessageType::NONE;
    }
    
    while (data_ < data_end_) {
      if (buffer_.size() == 0) {  // Looking for SYNC0
	if (*data_ == novatel::SYNC_0) {
	  buffer_.push_back(*data_);
	}
	++data_;
      } else if (buffer_.size() == 1) {  // Looking for SYNC1
	if (*data_ == novatel::SYNC_1) {
	  buffer_.push_back(*data_++);
	} else {
	  buffer_.clear();
	}
      } else if (buffer_.size() == 2) {  // Looking for SYNC2
	switch (*data_) {
        case novatel::SYNC_2_LONG_HEADER:
          buffer_.push_back(*data_++);
          header_length_ = sizeof(novatel::LongHeader);
          break;
        case novatel::SYNC_2_SHORT_HEADER:
          buffer_.push_back(*data_++);
          header_length_ = sizeof(novatel::ShortHeader);
          break;
        default:
          buffer_.clear();
	}
      } else if (header_length_ > 0) {  // Working on header.
	if (buffer_.size() < header_length_) {
	  buffer_.push_back(*data_++);
	} else {
	  if (header_length_ == sizeof(novatel::LongHeader)) {
	    total_length_ = header_length_ + novatel::CRC_LENGTH +
	      reinterpret_cast<novatel::LongHeader*>(buffer_.data())
	      ->message_length;
	  } else if (header_length_ == sizeof(novatel::ShortHeader)) {
	    total_length_ =
              header_length_ + novatel::CRC_LENGTH +
              reinterpret_cast<novatel::ShortHeader*>(buffer_.data())
	      ->message_length;
	  } else {
	    ROS_ERROR("Incorrect header_length_. Should never reach here.");
	    buffer_.clear();
	  }
	  header_length_ = 0;
	}
      } else if (total_length_ > 0) {
	if (buffer_.size() < total_length_) {  // Working on body.
	  buffer_.push_back(*data_++);
	  continue;
	}
	MessageType type = prepare_message(message_ptr);
	buffer_.clear();
	total_length_ = 0;
	if (type != MessageType::NONE) {
	  return type;
	}
      }
    }
    return MessageType::NONE;
  }

  bool NovatelParser::check_crc() {
    size_t l = buffer_.size() - novatel::CRC_LENGTH;
    return crc32_block(buffer_.data(), l) ==
      *reinterpret_cast<uint32_t*>(buffer_.data() + l);
  }
  
  Parser::MessageType NovatelParser::prepare_message(MessagePtr& message_ptr) {
    if (!check_crc()) {
      ROS_ERROR("CRC check failed.");
      return MessageType::NONE;
    }

    uint8_t* message = nullptr;
    novatel::MessageId message_id;
    uint16_t message_length;
    uint16_t gps_week;
    uint32_t gps_millisecs;
    if (buffer_[2] == novatel::SYNC_2_LONG_HEADER) {
      auto header = reinterpret_cast<const novatel::LongHeader*>(buffer_.data());
      message = buffer_.data() + sizeof(novatel::LongHeader);
      gps_week = header->gps_week;
      gps_millisecs = header->gps_millisecs;
      message_id = header->message_id;
      message_length = header->message_length;
    } else {
      auto header = reinterpret_cast<const novatel::ShortHeader*>(buffer_.data());
      message = buffer_.data() + sizeof(novatel::ShortHeader);
      gps_week = header->gps_week;
      gps_millisecs = header->gps_millisecs;
      message_id = header->message_id;
      message_length = header->message_length;
    }
    switch (message_id) {
    case novatel::BESTGNSSPOS:
    case novatel::BESTPOS:
    case novatel::PSRPOS:
      // ROS_ERROR_COND(message_length != sizeof(novatel::BestPos), "Incorrect
      // message_length");
      if (message_length != sizeof(novatel::BestPos)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }
      if (handle_best_pos(reinterpret_cast<novatel::BestPos*>(message),
                          gps_week, gps_millisecs)) {
        message_ptr = &gnss_;
        return MessageType::GNSS;
      }
      break;
      
    case novatel::BESTGNSSVEL:
    case novatel::BESTVEL:
    case novatel::PSRVEL:
      // ROS_ERROR_COND(message_length != sizeof(novatel::BestVel), "Incorrect
      // message_length");
      if (message_length != sizeof(novatel::BestVel)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }
      if (handle_best_vel(reinterpret_cast<novatel::BestVel*>(message),
                          gps_week, gps_millisecs)) {
        message_ptr = &gnss_;
        return MessageType::GNSS;
      }
      break;
      
    case novatel::CORRIMUDATA:
    case novatel::CORRIMUDATAS:
      // ROS_ERROR_COND(message_length != sizeof(novatel::CorrImuData),
      // "Incorrect message_length");
      if (message_length != sizeof(novatel::CorrImuData)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }
      
      if (handle_corr_imu_data(
              reinterpret_cast<novatel::CorrImuData*>(message))) {
        message_ptr = &ins_;
        return MessageType::INS;
      }
      break;
      
    case novatel::INSCOV:
    case novatel::INSCOVS:
      // ROS_ERROR_COND(message_length != sizeof(novatel::InsCov), "Incorrect
      // message_length");
      if (message_length != sizeof(novatel::InsCov)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }
      
      if (handle_ins_cov(reinterpret_cast<novatel::InsCov*>(message))) {
        message_ptr = &ins_;
        return MessageType::INS;
      }
      break;
      
    case novatel::INSPVA:
    case novatel::INSPVAS:
      // ROS_ERROR_COND(message_length != sizeof(novatel::InsPva), "Incorrect
      // message_length");
      if (message_length != sizeof(novatel::InsPva)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }
      
      if (handle_ins_pva(reinterpret_cast<novatel::InsPva*>(message))) {
        message_ptr = &ins_;
        return MessageType::INS;
      }
      break;
      
    case novatel::RAWIMUX:
    case novatel::RAWIMUSX:
      // ROS_ERROR_COND(message_length != sizeof(novatel::RawImuX), "Incorrect
      // message_length");
      if (message_length != sizeof(novatel::RawImuX)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }
      
      if (handle_raw_imu_x(reinterpret_cast<novatel::RawImuX*>(message))) {
        message_ptr = &imu_;
        return MessageType::IMU;
      }
      break;
      
    default:
      break;
    }
    return MessageType::NONE;
  }

  bool NovatelParser::handle_best_pos(const novatel::BestPos* pos,
                                    uint16_t gps_week, uint32_t gps_millisecs) {
    gnss_.mutable_position()->set_lon(pos->longitude);
    gnss_.mutable_position()->set_lat(pos->latitude);
    gnss_.mutable_position()->set_height(pos->height_msl + pos->undulation);
    gnss_.mutable_position_std_dev()->set_x(pos->longitude_std_dev);
    gnss_.mutable_position_std_dev()->set_y(pos->latitude_std_dev);
    gnss_.mutable_position_std_dev()->set_z(pos->height_std_dev);
    gnss_.set_num_sats(pos->num_sats_in_solution);
    if (solution_status_ != pos->solution_status) {
      solution_status_ = pos->solution_status;
      ROS_INFO_STREAM("Solution status: " << static_cast<int>(solution_status_));
    }
    if (position_type_ != pos->position_type) {
      position_type_ = pos->position_type;
      ROS_INFO_STREAM("Position type: " << static_cast<int>(position_type_));
    }
    gnss_.set_solution_status(static_cast<uint32_t>(pos->solution_status));
    if (pos->solution_status == novatel::SolutionStatus::SOL_COMPUTED) {
      gnss_.set_position_type(static_cast<uint32_t>(pos->position_type));
      switch (pos->position_type) {
      case novatel::SolutionType::SINGLE:
      case novatel::SolutionType::INS_PSRSP:
        gnss_.set_type(gnss_driver::pb::Gnss::SINGLE);
        break;
      case novatel::SolutionType::PSRDIFF:
      case novatel::SolutionType::WAAS:
      case novatel::SolutionType::INS_SBAS:
        gnss_.set_type(gnss_driver::pb::Gnss::PSRDIFF);
        break;
      case novatel::SolutionType::FLOATCONV:
      case novatel::SolutionType::L1_FLOAT:
      case novatel::SolutionType::IONOFREE_FLOAT:
      case novatel::SolutionType::NARROW_FLOAT:
      case novatel::SolutionType::RTK_DIRECT_INS:
      case novatel::SolutionType::INS_RTKFLOAT:
        gnss_.set_type(gnss_driver::pb::Gnss::RTK_FLOAT);
        break;
      case novatel::SolutionType::WIDELANE:
      case novatel::SolutionType::NARROWLANE:
      case novatel::SolutionType::L1_INT:
      case novatel::SolutionType::WIDE_INT:
      case novatel::SolutionType::NARROW_INT:
      case novatel::SolutionType::INS_RTKFIXED:
        gnss_.set_type(gnss_driver::pb::Gnss::RTK_INTEGER);
        break;
      case novatel::SolutionType::OMNISTAR:
      case novatel::SolutionType::INS_OMNISTAR:
      case novatel::SolutionType::INS_OMNISTAR_HP:
      case novatel::SolutionType::INS_OMNISTAR_XP:
      case novatel::SolutionType::OMNISTAR_HP:
      case novatel::SolutionType::OMNISTAR_XP:
      case novatel::SolutionType::PPP_CONVERGING:
      case novatel::SolutionType::PPP:
      case novatel::SolutionType::INS_PPP_CONVERGING:
      case novatel::SolutionType::INS_PPP:
        gnss_.set_type(gnss_driver::pb::Gnss::PPP);
        break;
      case novatel::SolutionType::PROPOGATED:
        gnss_.set_type(gnss_driver::pb::Gnss::PROPAGATED);
        break;
      default:
        gnss_.set_type(gnss_driver::pb::Gnss::INVALID);
      }
    } else {
      gnss_.set_type(gnss_driver::pb::Gnss::INVALID);
      gnss_.set_position_type(0);
    }
    if (pos->datum_id != novatel::DatumId::WGS84) {
      ROS_ERROR_STREAM_THROTTLE(5, "Unexpected Datum Id: " << static_cast<int>(pos->datum_id));
    }
    
    double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
    if (gnss_.measurement_time() != seconds) {
      gnss_.set_measurement_time(seconds);
      return false;
    }
    return true;
  }
  
  bool NovatelParser::handle_best_vel(const novatel::BestVel* vel,
				      uint16_t gps_week, uint32_t gps_millisecs) {
    if (velocity_type_ != vel->velocity_type) {
      velocity_type_ = vel->velocity_type;
      ROS_INFO_STREAM("Velocity type: " << static_cast<int>(velocity_type_));
    }
    if (!gnss_.has_velocity_latency() ||
	gnss_.velocity_latency() != vel->latency) {
      ROS_INFO_STREAM("Velocity latency: " << static_cast<int>(vel->latency));
      gnss_.set_velocity_latency(vel->latency);
    }
    double yaw = azimuth_deg_to_yaw_rad(vel->track_over_ground);
    gnss_.mutable_linear_velocity()->set_x(vel->horizontal_speed * cos(yaw));
    gnss_.mutable_linear_velocity()->set_y(vel->horizontal_speed * sin(yaw));
    gnss_.mutable_linear_velocity()->set_z(vel->vertical_speed);
    
    double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
    if (gnss_.measurement_time() != seconds) {
      gnss_.set_measurement_time(seconds);
      return false;
    }
    return true;
  }

  bool NovatelParser::handle_corr_imu_data(const novatel::CorrImuData* imu) {
    rfu_to_flu(imu->x_velocity_change * imu_measurement_hz_,
	       imu->y_velocity_change * imu_measurement_hz_,
	       imu->z_velocity_change * imu_measurement_hz_,
	       ins_.mutable_linear_acceleration());
    rfu_to_flu(imu->x_angle_change * imu_measurement_hz_,
	       imu->y_angle_change * imu_measurement_hz_,
	       imu->z_angle_change * imu_measurement_hz_,
	       ins_.mutable_angular_velocity());
    
    double seconds = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
    if (ins_.measurement_time() != seconds) {
      ins_.set_measurement_time(seconds);
      return false;
    }
    ins_.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    return true;
  }

  bool NovatelParser::handle_ins_cov(const novatel::InsCov* cov) {
    for (int i = 0; i < 9; ++i) {
      ins_.set_position_covariance(i, cov->position_covariance[i]);
      ins_.set_euler_angles_covariance(INDEX[i], (DEG_TO_RAD * DEG_TO_RAD) *
				       cov->attitude_covariance[i]);
      ins_.set_linear_velocity_covariance(i, cov->velocity_covariance[i]);
    }
    return false;
  }
  
  bool NovatelParser::handle_ins_pva(const novatel::InsPva* pva) {
    if (ins_status_ != pva->status) {
      ins_status_ = pva->status;
      ROS_INFO_STREAM("INS status: " << static_cast<int>(ins_status_));
    }
    ins_.mutable_position()->set_lon(pva->longitude);
    ins_.mutable_position()->set_lat(pva->latitude);
    ins_.mutable_position()->set_height(pva->height);
    ins_.mutable_euler_angles()->set_x(pva->roll * DEG_TO_RAD);
    ins_.mutable_euler_angles()->set_y(-pva->pitch * DEG_TO_RAD);
    ins_.mutable_euler_angles()->set_z(azimuth_deg_to_yaw_rad(pva->azimuth));
    ins_.mutable_linear_velocity()->set_x(pva->east_velocity);
    ins_.mutable_linear_velocity()->set_y(pva->north_velocity);
    ins_.mutable_linear_velocity()->set_z(pva->up_velocity);
    
    switch (pva->status) {
    case novatel::InsStatus::ALIGNMENT_COMPLETE:
    case novatel::InsStatus::SOLUTION_GOOD:
      ins_.set_type(gnss_driver::pb::Ins::GOOD);
      break;
    case novatel::InsStatus::ALIGNING:
    case novatel::InsStatus::HIGH_VARIANCE:
    case novatel::InsStatus::SOLUTION_FREE:
      ins_.set_type(gnss_driver::pb::Ins::CONVERGING);
      break;
    default:
      ins_.set_type(gnss_driver::pb::Ins::INVALID);
    }
    
    double seconds = pva->gps_week * SECONDS_PER_WEEK + pva->gps_seconds;
    if (ins_.measurement_time() != seconds) {
      ins_.set_measurement_time(seconds);
      return false;
    }
    
    ins_.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    return true;
  }

  bool NovatelParser::handle_raw_imu_x(const novatel::RawImuX* imu) {
    if (imu->imu_error != 0) {
      ROS_WARN_STREAM("IMU error. Status: " << std::hex << std::showbase
		      << imu->imuStatus);
    }
    if (is_zero(gyro_scale_)) {
      novatel::ImuParameter param = novatel::get_imu_parameter(imu->imu_type);
      ROS_INFO_STREAM("IMU type: " << static_cast<unsigned>(imu->imu_type) << "; "
		      << "Gyro scale: " << param.gyro_scale << "; "
		      << "Accel scale: " << param.accel_scale << "; "
		      << "Sampling rate: " << param.sampling_rate_hz
		      << ".");
      
      if (is_zero(param.sampling_rate_hz)) {
	ROS_ERROR_STREAM_THROTTLE(5, "Unsupported IMU type: " << static_cast<int>(imu->imu_type));
	return false;
      }
      gyro_scale_ = param.gyro_scale * param.sampling_rate_hz;
      accel_scale_ = param.accel_scale * param.sampling_rate_hz;
      imu_measurement_hz_ = param.sampling_rate_hz;
      imu_measurement_span_ = 1.0 / param.sampling_rate_hz;
      imu_.set_measurement_span(imu_measurement_span_);
    }
    
    double time = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
    if (imu_measurement_time_previous_ > 0.0 &&
	fabs(time - imu_measurement_time_previous_ - imu_measurement_span_) >
	1e-4) {
      ROS_WARN_STREAM("Unexpected delay between two IMU measurements at: "
		      << time - imu_measurement_time_previous_);
    }
    imu_.set_measurement_time(time);
    switch (imu_frame_mapping_) {
    case 5:  // Default mapping.
      rfu_to_flu(imu->x_velocity_change * accel_scale_,
                 -imu->y_velocity_change_neg * accel_scale_,
                 imu->z_velocity_change * accel_scale_,
                 imu_.mutable_linear_acceleration());
      rfu_to_flu(imu->x_angle_change * gyro_scale_,
                 -imu->y_angle_change_neg * gyro_scale_,
                 imu->z_angle_change * gyro_scale_,
                 imu_.mutable_angular_velocity());
      break;
    case 6:
      rfu_to_flu(-imu->y_velocity_change_neg * accel_scale_,
                 imu->x_velocity_change * accel_scale_,
                 -imu->z_velocity_change * accel_scale_,
                 imu_.mutable_linear_acceleration());
      rfu_to_flu(-imu->y_angle_change_neg * gyro_scale_,
                 imu->x_angle_change * gyro_scale_,
                 -imu->z_angle_change * gyro_scale_,
                 imu_.mutable_angular_velocity());
      break;
    default:
      ROS_ERROR_STREAM_THROTTLE(5, "Unsupported IMU frame mapping: " << imu_frame_mapping_);
    }
    imu_measurement_time_previous_ = time;
    return true;
  }
  
}  // namespace gnss_driver
