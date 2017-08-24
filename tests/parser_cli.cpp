// A command-line interface (CLI) tool of parser. It parses a binary log file. It is supposed to be
// used for verifying if the parser works properly.

#include <fstream>
#include <iostream>
#include <memory>
#include <ros/ros.h>

#include "gnss/parser.h"
#include "gnss/stream.h"

namespace gnss_driver {
  
  constexpr size_t BUFFER_SIZE = 128;
  
  void Parse(const char* filename, char parser_type) {
    std::ios::sync_with_stdio(false);
    std::ifstream f(filename, std::ifstream::binary);
    char b[BUFFER_SIZE];
    std::unique_ptr<Parser> p;
    switch (parser_type) {
    case 'n':
      p.reset(Parser::create_novatel());
      break;
    default:
      std::cout << "Log type should be either 'n' or 'u'" << std::endl;
      return;
    }
    while (f) {
      f.read(b, BUFFER_SIZE);
      p->update(reinterpret_cast<uint8_t*>(b), f.gcount());
      for (;;) {
	MessagePtr msg_ptr;
	if (p->get_message(msg_ptr) == Parser::MessageType::NONE) {
	  break;
	}
	// msg_ptr->PrintDebugString();
      }
    }
  }
  
}  // namespace gnss_driver

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "Usage: " << argv[0] << " filename [n|u]" << std::endl;
    return 0;
  }
  
  ros::Time::init();
  gnss_driver::Parse(argv[1], argv[2][0]);
  return 0;
}
