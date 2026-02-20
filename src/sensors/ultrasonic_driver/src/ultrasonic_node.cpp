#include <chrono>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;

class UltrasonicSensorNode : public rclcpp::Node
{
public:
  UltrasonicSensorNode() : Node("ultrasonic_sensor_node")
  {
    // Check /dev/ttyTHS1 if you are using Jetson GPIO pins, 
    // or /dev/ttyUSB0 for a USB adapter.
    serial_port_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    
    if (serial_port_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", std::strerror(errno));
      return;
    }

    setup_serial();
    
    publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/ultrasonic/distance", 10);
    
    // 10Hz polling rate (100ms) matches your Arduino loop delay
    timer_ = this->create_wall_timer(100ms, std::bind(&UltrasonicSensorNode::read_sensor, this));
    
    RCLCPP_INFO(this->get_logger(), "Ultrasonic Node Started. Port: /dev/ttyUSB0");
  }

  ~UltrasonicSensorNode() {
    if (serial_port_ >= 0) close(serial_port_);
  }

private:
  void setup_serial()
  {
    struct termios tty;
    if (tcgetattr(serial_port_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed");
      return;
    }

    // Set Baud Rate
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8N1 Mode
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 bits
    tty.c_cflag |= (CLOCAL | CREAD); 

    // Raw Mode (Non-canonical)
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    tty.c_oflag &= ~OPOST;

    // Blocking behavior: Wait up to 100ms for at least 1 byte
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; // 1 = 100ms

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed");
    }
    
    tcflush(serial_port_, TCIFLUSH);
  }

  void read_sensor()
  {
    // 1. Flush old data to ensure we get a fresh reading
    tcflush(serial_port_, TCIFLUSH);

    // 2. Trigger the sensor
    uint8_t cmd = 0x55;
    if (write(serial_port_, &cmd, 1) != 1) {
      RCLCPP_WARN(this->get_logger(), "Write failed");
      return;
    }

    // 3. Wait for response (Sensor needs time to ping and calculate)
    // Most sensors respond within 30-50ms
    std::this_thread::sleep_for(60ms);

    uint8_t buffer[4];
    uint8_t header = 0;
    
    // 4. Look for the start byte (0xFF)
    bool found_header = false;
    for (int i = 0; i < 10; ++i) { // Try a few times to find the header
        if (read(serial_port_, &header, 1) > 0) {
            if (header == 0xFF) {
                found_header = true;
                break;
            }
        }
    }

    if (!found_header) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Header 0xFF not found");
      return;
    }

    // 5. Read the remaining 3 bytes (Data_H, Data_L, Checksum)
    ssize_t n = read(serial_port_, buffer, 3);
    if (n == 3) {
      uint8_t high = buffer[0];
      uint8_t low  = buffer[1];
      uint8_t sum  = buffer[2];
      
      // Calculate Checksum: (Header + High + Low) & 0x00FF
      uint8_t calc_sum = (0xFF + high + low) & 0xFF;

      if (sum == calc_sum) {
        int distance_mm = (high << 8) | low;
        publish_range(distance_mm);
      } else {
        RCLCPP_WARN(this->get_logger(), "Checksum error! Calculated: %02X, Received: %02X", calc_sum, sum);
      }
    }
  }

  void publish_range(int mm)
  {
    auto msg = sensor_msgs::msg::Range();
    msg.header.stamp = this->now();
    msg.header.frame_id = "ultrasonic_link";
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view = 0.5235; // Approx 30 degrees in radians
    msg.min_range = 0.03;       // 3cm
    msg.max_range = 4.0;        // 4m
    msg.range = static_cast<float>(mm) / 1000.0f;

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Distance: %d mm", mm);
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