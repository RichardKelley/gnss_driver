#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>

#include <ros/ros.h>

#include "gnss/stream.h"
#include "tcp_stream.h"

namespace gnss_driver {

  TcpStream::TcpStream(const char* address, uint16_t port, uint32_t timeout_usec)
    : sockfd_(-1), errno_(0) {
    peer_addr_ = inet_addr(address);
    peer_port_ = htons(port);
    timeout_usec_ = timeout_usec;
  }
  
  TcpStream::~TcpStream() { this->close(); }
  
  void TcpStream::open() {
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) {
      // error
      ROS_ERROR_STREAM("create socket failed, errno: " << errno << ", "
		       << strerror(errno));
      return;
    }
    
    sockfd_ = fd;
  }
  
  bool TcpStream::init_socket() {
    if (sockfd_ < 0) {
      return false;
    }
    
    // block or not block
    if (timeout_usec_ != 0) {
      int flags = fcntl(sockfd_, F_GETFL, 0);
      if (flags == -1) {
	::close(sockfd_);
	ROS_ERROR_STREAM("fcntl get flag failed, error: " << strerror(errno)
			 << ".");
	return false;
      }
      
      if (fcntl(sockfd_, F_SETFL, flags & ~O_NONBLOCK) == -1) {
	::close(sockfd_);
	ROS_ERROR_STREAM("fcntl set block failed, error: " << strerror(errno)
			 << ".");
	return false;
      }
      
      timeval block_to = {timeout_usec_ / 1000000, timeout_usec_ % 1000000};
      if (setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &block_to,
		     sizeof(block_to)) < 0) {
	::close(sockfd_);
	ROS_ERROR_STREAM("setsockopt set rcv timeout failed, error: "
			 << strerror(errno) << ".");
	return false;
      }
      
      if (setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, &block_to,
		     sizeof(block_to)) < 0) {
	::close(sockfd_);
	ROS_ERROR_STREAM("setsockopt set snd timeout failed, error: "
			 << strerror(errno) << ".");
	return false;
      }
    } else {
      int flags = fcntl(sockfd_, F_GETFL, 0);
      if (flags == -1) {
	::close(sockfd_);
	ROS_ERROR_STREAM("fcntl get flag failed, error: " << strerror(errno)
			 << ".");
	return false;
      }
      
      if (fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) == -1) {
	::close(sockfd_);
	ROS_ERROR_STREAM("fcntl set non block failed, error: " << strerror(errno)
			 << ".");
	return false;
      }
    }
    
    // disable Nagle
    int ret = 0;
    int enable = 1;
    ret = setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (void*)&enable,
		     sizeof(enable));
    if (ret == -1) {
      ::close(sockfd_);
      ROS_ERROR_STREAM("setsockopt disable Nagle failed, errno: "
		       << errno << ", " << strerror(errno));
      return false;
    }
    
    return true;
  }
  
  void TcpStream::close() {
    if (sockfd_ > 0) {
      ::close(sockfd_);
      sockfd_ = -1;
      status_ = Stream::Status::DISCONNECTED;
    }
  }
  
  bool TcpStream::connect() {
    if (sockfd_ < 0) {
      this->open();
      if (sockfd_ < 0) {
	// error
	return false;
      }
    }
    
    if (status_ == Stream::Status::CONNECTED) {
      return true;
    }
    
    fd_set fds;
    timeval timeo = {10, 0};
    int ret = 0;
    sockaddr_in peer_addr;
    
    bzero(&peer_addr, sizeof(peer_addr));
    peer_addr.sin_family = AF_INET;
    peer_addr.sin_port = peer_port_;
    peer_addr.sin_addr.s_addr = peer_addr_;
    
    int fd_flags = fcntl(sockfd_, F_GETFL);
    if (fd_flags < 0 || fcntl(sockfd_, F_SETFL, fd_flags | O_NONBLOCK) < 0) {
      ROS_ERROR_STREAM("Failed to set noblock, error: " << strerror(errno));
      return false;
    }
    
    while ((ret = ::connect(sockfd_, reinterpret_cast<sockaddr*>(&peer_addr),
			    sizeof(peer_addr))) < 0) {
      if (errno == EINTR) {
	ROS_INFO("Tcp connect return EINTR.");
	continue;
      } else {
	if ((errno != EISCONN) && (errno != EINPROGRESS) && (errno != EALREADY)) {
	  status_ = Stream::Status::ERROR;
	  errno_ = errno;
	  ROS_ERROR_STREAM("Connect failed, error: " << strerror(errno));
	  return false;
	}
	
	FD_ZERO(&fds);
	FD_SET(sockfd_, &fds);
	ret = select(sockfd_ + 1, NULL, &fds, NULL, &timeo);
	if (ret < 0) {
	  status_ = Stream::Status::ERROR;
	  errno_ = errno;
	  ROS_ERROR_STREAM("Wait connect failed, error: " << strerror(errno));
	  return false;
	} else if (ret == 0) {
	  ROS_INFO("Tcp connect timeout.");
	  return false;
	} else if (FD_ISSET(sockfd_, &fds)) {
	  int error = 0;
	  socklen_t len = sizeof(int);
	  
	  if (getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &error, &len) < 0) {
	    status_ = Stream::Status::ERROR;
	    errno_ = errno;
	    ROS_ERROR_STREAM("Getsockopt failed, error: " << strerror(errno));
	    return false;
	  }
	  if (error != 0) {
	    status_ = Stream::Status::ERROR;
	    errno_ = errno;
	    ROS_ERROR_STREAM("Socket error: " << strerror(errno));
	    return false;
	  }
	  
	  // connect successfully
	  break;
	} else {
	  status_ = Stream::Status::ERROR;
	  errno_ = errno;
	  ROS_ERROR("Should not be here.");
	  return false;
	}
      }
    }
    
    if (!init_socket()) {
      close();
      status_ = Stream::Status::ERROR;
      errno_ = errno;
      ROS_ERROR("Failed to init socket.");
      return false;
    }
    ROS_INFO("Tcp connect success.");
    status_ = Stream::Status::CONNECTED;
    login();
    return true;
  }
  
  bool TcpStream::disconnect() {
    if (sockfd_ < 0) {
      // not open
      return false;
    }
    
    this->close();
    return true;
  }
  
  size_t TcpStream::read(uint8_t* buffer, size_t max_length) {
    ssize_t ret = 0;
    
    if (status_ != Stream::Status::CONNECTED) {
      disconnect();
      connect();
      if (status_ != Stream::Status::CONNECTED) {
	return 0;
      }
    }
    
    if (!readable(10000)) {
      return 0;
    }
    
    while ((ret = ::recv(sockfd_, buffer, max_length, 0)) < 0) {
      if (errno == EINTR) {
	continue;
      } else {
	// error
	if (errno != EAGAIN) {
	  status_ = Stream::Status::ERROR;
	  errno_ = errno;
	  ROS_ERROR("Read errno %d, error %s.", errno, strerror(errno));
	}
      }
      
      return 0;
    }
    
    if (ret == 0) {
      status_ = Stream::Status::ERROR;
      errno_ = errno;
      ROS_ERROR("Remote closed.");
      disconnect();
      if (connect()) {
	ROS_INFO("Reconnect tcp success.");
      }
    }
    
    return ret;
  }
  
  size_t TcpStream::write(const uint8_t* buffer, size_t length) {
    size_t total_nsent = 0;
    
    if (status_ != Stream::Status::CONNECTED) {
      disconnect();
      connect();
      if (status_ != Stream::Status::CONNECTED) {
	return 0;
      }
    }
    
    while (length > 0) {
      ssize_t nsent = ::send(sockfd_, buffer, length, 0);
      if (nsent < 0) {
	if (errno == EINTR) {
	  continue;
	} else {
	  // error
	  if (errno == EPIPE || errno == ECONNRESET) {
	    status_ = Stream::Status::DISCONNECTED;
	    errno_ = errno;
	  } else if (errno != EAGAIN) {
	    status_ = Stream::Status::ERROR;
	    errno_ = errno;
	  }
	  return total_nsent;
	}
      }
      
      total_nsent += nsent;
      length -= nsent;
      buffer += nsent;
    }
    
    return total_nsent;
  }
  
  bool TcpStream::readable(uint32_t timeout_us) {
    // Setup a select call to block for serial data or a timeout
    timespec timeout_ts;
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sockfd_, &readfds);
    
    timeout_ts.tv_sec = timeout_us / 1000000;
    timeout_ts.tv_nsec = (timeout_us % 1000000) * 1000;
    int r = pselect(sockfd_ + 1, &readfds, NULL, NULL, &timeout_ts, NULL);
    if (r < 0) {
      status_ = Stream::Status::ERROR;
      errno_ = errno;
      ROS_ERROR("Failed to wait tcp data: %d, %s", errno, strerror(errno));
      return false;
    } else if (r == 0 || !FD_ISSET(sockfd_, &readfds)) {
      return false;
    }
    // Data available to read.
    return true;
  }
  
  Stream* Stream::create_tcp(const char* address, uint16_t port,
			     uint32_t timeout_usec) {
    return new TcpStream(address, port, timeout_usec);
  }
  
}  // namespace gnss_driver
