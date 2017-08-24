#ifndef GNSS_DRIVER_TCP_STREAM_H
#define GNSS_DRIVER_TCP_STREAM_H

namespace gnss_driver {

class TcpStream : public Stream {
  typedef uint16_t be16_t;
  typedef uint32_t be32_t;

 public:
  TcpStream(const char *address, uint16_t port, uint32_t timeout_usec);
  ~TcpStream();

  virtual bool connect();
  virtual bool disconnect();
  virtual size_t read(uint8_t *buffer, size_t max_length);
  virtual size_t write(const uint8_t *data, size_t length);

 private:
  bool readable(uint32_t timeout_us);
  TcpStream() {}
  void open();
  void close();
  bool init_socket();
  be16_t peer_port_ = 0;
  be32_t peer_addr_ = 0;
  uint32_t timeout_usec_ = 0;
  int sockfd_ = -1;
  int errno_ = 0;
};

}  // namespace gnss_driver

#endif
