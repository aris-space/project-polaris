#include <chrono>
#include <memory>
#include <string>
#include <thread>
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

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed while configuring serial port");
      return;
    }

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

    // Give the sensor a moment to respond
    std::this_thread::sleep_for(10ms);

    uint8_t buffer[4];
    ssize_t bytes_read = read(serial_port_, buffer, 4);
    if (bytes_read != 4)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Incomplete sensor response, bytes_read=%ld",
          static_cast<long>(bytes_read));
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