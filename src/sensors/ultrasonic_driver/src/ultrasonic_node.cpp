#include <chrono>
#include <cerrno>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp" // Better than a raw Int32 for distance

using namespace std::chrono_literals;

class UltrasonicSensorNode : public rclcpp::Node
{
public:
  UltrasonicSensorNode() : Node("ultrasonic_sensor_node")
  {
    // 1. Setup Serial Port (Jetson Nano Port 1 is usually /dev/ttyTHS1)
    serial_port_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (serial_port_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port /dev/ttyUSB0");
      return;
    }

    setup_serial();
    RCLCPP_INFO(this->get_logger(), "Serial port initialized for Ultrasonic Sensor");

    // 2. Setup Publisher (Using Range message for ROS 2 standards)
    publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/ultrasonic/distance", 10);

    // 3. Setup Timer (Replaces the Arduino loop)
    timer_ = this->create_wall_timer(100ms, std::bind(&UltrasonicSensorNode::read_sensor, this));
  }

  ~UltrasonicSensorNode()
  {
    if (serial_port_ >= 0)
      close(serial_port_);
  }

private:
  bool read_exact(uint8_t * buffer, size_t expected_len, std::chrono::milliseconds timeout)
  {
    size_t total_read = 0;
    auto deadline = std::chrono::steady_clock::now() + timeout;

    while (total_read < expected_len && std::chrono::steady_clock::now() < deadline)
    {
      ssize_t n = read(serial_port_, buffer + total_read, expected_len - total_read);
      if (n > 0)
      {
        total_read += static_cast<size_t>(n);
        continue;
      }

      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
      {
        RCLCPP_WARN(this->get_logger(), "Serial read error: %s", std::strerror(errno));
        return false;
      }

      std::this_thread::sleep_for(1ms);
    }

    if (total_read != expected_len)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Incomplete sensor response, bytes_read=%zu/%zu",
          total_read,
          expected_len);
      return false;
    }

    return true;
  }

  void setup_serial()
  {
    struct termios tty;
    if (tcgetattr(serial_port_, &tty) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed while configuring serial port");
      return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD); // Ignore modem lines, enable receiver
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 bit chars
    tty.c_cflag &= ~PARENB; // no parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit

    tty.c_lflag = 0; // raw mode (non-canonical)
    tty.c_oflag = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; // 100 ms max per read call

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed while configuring serial port");
      return;
    }

    tcflush(serial_port_, TCIFLUSH);

    RCLCPP_DEBUG(this->get_logger(), "Serial port configured: 115200 8N1");
  }

  void read_sensor()
  {
    if (serial_port_ < 0)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Serial port is not available, skipping read");
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Reading ultrasonic sensor data");

    uint8_t COM = 0x55;
    ssize_t bytes_written = write(serial_port_, &COM, 1); // Trigger the sensor
    if (bytes_written != 1)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to trigger ultrasonic sensor, bytes_written=%ld", static_cast<long>(bytes_written));
      return;
    }
    RCLCPP_DEBUG(this->get_logger(), "Triggered ultrasonic sensor");

    // Arduino-equivalent timing: delay(100) then delay(4)
    std::this_thread::sleep_for(100ms);

    int available_bytes = 0;
    if (ioctl(serial_port_, FIONREAD, &available_bytes) == -1)
    {
      RCLCPP_WARN(this->get_logger(), "ioctl(FIONREAD) failed: %s", std::strerror(errno));
      return;
    }

    if (available_bytes <= 0)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "No sensor bytes available after trigger");
      return;
    }

    std::this_thread::sleep_for(4ms);

    uint8_t buffer[4];
    if (!read_exact(buffer, 1, 20ms))
    {
      return;
    }

    if (buffer[0] != 0xFF)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Invalid frame header: 0x%02X",
          buffer[0]);
      return;
    }

    if (!read_exact(buffer + 1, 3, 40ms))
    {
      return;
    }

    uint8_t checksum = buffer[0] + buffer[1] + buffer[2];
    if (buffer[3] != checksum)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Checksum mismatch: recv=0x%02X expected=0x%02X",
          buffer[3],
          checksum);
      return;
    }

    int distance_mm = (buffer[1] << 8) + buffer[2];

    auto message = sensor_msgs::msg::Range();
    message.header.stamp = this->now();
    message.header.frame_id = "ultrasonic_link";
    message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    message.range = static_cast<float>(distance_mm) / 1000.0f; // Convert to Meters

    RCLCPP_INFO(this->get_logger(), "Distance: %d mm", distance_mm);
    publisher_->publish(message);

    RCLCPP_DEBUG(
        this->get_logger(),
        "Published range: %.3f m (raw bytes: [%02X %02X %02X %02X])",
        message.range,
        buffer[0],
        buffer[1],
        buffer[2],
        buffer[3]);
  }

  int serial_port_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicSensorNode>());
  rclcpp::shutdown();
  return 0;
}