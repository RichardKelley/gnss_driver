#ifndef DRIVERS_GNSS_UTILS_H
#define DRIVERS_GNSS_UTILS_H

#include <string>

#include "proto/config.pb.h"

namespace gnss_driver {

// TODO figure out how to handle config information.
  
bool parse_config_text(const std::string &filename, config::Config *config);

std::string encode_base64(const std::string &in);

}  // namespace gnss_driver

#endif  // DRIVERS_GNSS_UTILS_H
