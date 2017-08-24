#include <fcntl.h>
#include <unistd.h>
#include <string>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "gnss/utils.h"

namespace gnss_driver {
namespace {

const char TABLE[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

const char* triplet_base64(int triplet) {
  static char result[4];
  result[0] = TABLE[(triplet >> 18) & 0x3f];
  result[1] = TABLE[(triplet >> 12) & 0x3f];
  result[2] = TABLE[(triplet >> 6) & 0x3f];
  result[3] = TABLE[triplet & 0x3f];
  return result;
}

}  // namespace

bool parse_config_text(const std::string& filename, config::Config* config) {
  int fd = open(filename.c_str(), O_RDONLY);
  if (-1 == fd) {
    return false;
  }

  google::protobuf::io::FileInputStream fs(fd);
  if (!::google::protobuf::TextFormat::Parse(&fs, config)) {
    close(fd);
    return false;
  }

  close(fd);
  return true;
}

std::string encode_base64(const std::string& in) {
  std::string out;
  if (in.empty()) {
    return out;
  }

  int in_size = in.size();

  out.reserve(((in_size - 1) / 3 + 1) * 4);

  int i = 2;
  for (; i < in_size; i += 3) {
    out.append(triplet_base64((in[i - 2] << 16) | (in[i - 1] << 8) | in[i]), 4);
  }
  if (i == in_size) {
    out.append(triplet_base64((in[i - 2] << 16) | (in[i - 1] << 8)), 3);
    out.push_back('=');
  } else if (i == in_size + 1) {
    out.append(triplet_base64(in[i - 2] << 16), 2);
    out.append("==");
  }
  return out;
}

}  // namespace gnss_driver
