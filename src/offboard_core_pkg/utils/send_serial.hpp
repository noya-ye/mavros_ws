#pragma once

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace offboard_core_pkg::utils {

class SendSerial {
public:
  SendSerial(const std::string &device, unsigned int baud_rate)
  : device_(device), baud_rate_(baud_rate) {}

  ~SendSerial() { close(); }

  SendSerial(const SendSerial &) = delete;
  SendSerial &operator=(const SendSerial &) = delete;

  bool open(std::string *error = nullptr) {
    close();
    const auto speed = baudConstant(baud_rate_);
    if (speed == 0) {
      return fail("unsupported baud rate: " + std::to_string(baud_rate_), error);
    }

    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      return fail(systemError("open " + device_), error);
    }

    termios options{};
    if (::tcgetattr(fd_, &options) != 0) {
      const auto message = systemError("tcgetattr " + device_);
      close();
      return fail(message, error);
    }

    ::cfmakeraw(&options);
    options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    options.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    options.c_cflag &= static_cast<tcflag_t>(~PARENB);
    options.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (::cfsetispeed(&options, speed) != 0 ||
        ::cfsetospeed(&options, speed) != 0 ||
        ::tcsetattr(fd_, TCSANOW, &options) != 0) {
      const auto message = systemError("configure " + device_);
      close();
      return fail(message, error);
    }

    return true;
  }

  void close() noexcept {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool isOpen() const noexcept { return fd_ >= 0; }

  bool send(const void *data, std::size_t size, std::string *error = nullptr) {
    if (fd_ < 0) {
      return fail("serial device is not open", error);
    }
    if (data == nullptr && size != 0) {
      return fail("serial data is null", error);
    }

    const auto *bytes = static_cast<const std::uint8_t *>(data);
    std::size_t sent = 0;
    while (sent < size) {
      const auto result = ::write(fd_, bytes + sent, size - sent);
      if (result < 0 && errno == EINTR) {
        continue;
      }
      if (result <= 0) {
        return fail(systemError("write " + device_), error);
      }
      sent += static_cast<std::size_t>(result);
    }
    return true;
  }

  bool send(const std::string &data, std::string *error = nullptr) {
    return send(data.data(), data.size(), error);
  }

  const std::string &device() const noexcept { return device_; }
  unsigned int baudRate() const noexcept { return baud_rate_; }

private:
  static speed_t baudConstant(unsigned int baud_rate) noexcept {
    switch (baud_rate) {
      case 1200: return B1200;
      case 2400: return B2400;
      case 4800: return B4800;
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
#ifdef B230400
      case 230400: return B230400;
#endif
      default: return 0;
    }
  }

  static std::string systemError(const std::string &operation) {
    return operation + ": " + std::strerror(errno);
  }

  static bool fail(const std::string &message, std::string *error) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  }

  std::string device_;
  unsigned int baud_rate_;
  int fd_{-1};
};

}  // namespace offboard_core_pkg::utils
