#include <chrono>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class UltrasonicSensorNode : public rclcpp::Node
{
public:
  UltrasonicSensorNode() : Node("ultrasonic_sensor_node")
  {
    serial_device_ = this->declare_parameter<std::string>("serial_device", "/dev/ultrasonic_front");

    // Open in Read/Write mode. O_NDELAY prevents the open call from blocking.
    serial_port_ = open(serial_device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    
    if (serial_port_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Could not open %s. Error: %s", serial_device_.c_str(), std::strerror(errno));
      RCLCPP_ERROR(this->get_logger(), "TIP: Try 'sudo chmod 666 %s'", serial_device_.c_str());
      return;
    }

    setup_serial();
    
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("ultrasonic/distance", 10);
    
    // 200ms timer = 5Hz frequency
    timer_ = this->create_wall_timer(200ms, std::bind(&UltrasonicSensorNode::read_sensor, this));
    
    RCLCPP_INFO(this->get_logger(), "Ultrasonic Node initialized on %s", serial_device_.c_str());
  }

  ~UltrasonicSensorNode() {
    if (serial_port_ >= 0) close(serial_port_);
  }

private:
  void setup_serial()
  {
    struct termios tty;
    if (tcgetattr(serial_port_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", std::strerror(errno));
      return;
    }

    // Set Baud Rate to 115200 (Matches your Arduino code)
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // Set hardware parameters: 8N1
    tty.c_cflag &= ~PARENB;        // No parity bit
    tty.c_cflag &= ~CSTOPB;        // Only one stop bit
    tty.c_cflag &= ~CSIZE;         // Clear size mask
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag |= (CLOCAL | CREAD); // Ignore modem lines, enable receiver

    // Disable canonical mode (we want raw bytes, not lines)
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    tty.c_oflag &= ~OPOST;

    // VMIN = 0, VTIME = 1: Read will return as soon as any data is received, 
    // or timeout after 100ms if nothing arrives.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; 

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", std::strerror(errno));
    }
    
    // Clear buffers
    tcflush(serial_port_, TCIOFLUSH);
  }

  void read_sensor()
  {
    // 1. Send Trigger Pulse (0x55)
    uint8_t trigger_byte = 0x55;
    if (write(serial_port_, &trigger_byte, 1) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port");
      return;
    }

    // 2. Wait for the sensor to process the ping and reply
    // On USB-to-TTL, 50-70ms is the "sweet spot" to ensure the buffer is filled
    std::this_thread::sleep_for(70ms);

    // 3. Search for the Header (0xFF)
    // We read one byte at a time until we find 0xFF to stay in sync
    uint8_t header = 0;
    int attempts = 0;
    bool found_header = false;

    while (attempts < 32) {
      if (read(serial_port_, &header, 1) > 0) {
        if (header == 0xFF) {
          found_header = true;
          break;
        }
      }
      attempts++;
    }

    if (!found_header) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for sensor data (Header 0xFF not found)...");
      return;
    }

    // 4. Read the payload (3 bytes: High, Low, Checksum)
    uint8_t data[3];
    ssize_t n = read(serial_port_, data, 3);
    
    if (n == 3) {
      uint8_t high = data[0];
      uint8_t low  = data[1];
      uint8_t received_sum = data[2];
      
      // Arduino Logic: Checksum = Header + High + Low
      uint8_t calculated_sum = (0xFF + high + low) & 0xFF;

      if (received_sum == calculated_sum) {
        int distance_mm = (high << 8) | low;
        float distance_m = static_cast<float>(distance_mm) / 1000.0f;
        auto msg = std_msgs::msg::Float32();
        msg.data = distance_m;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Distance: %.3f m", distance_m);
      } else {
        RCLCPP_WARN(this->get_logger(), "Checksum Failed!");
      }
    }
  }

  int serial_port_;
  std::string serial_device_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicSensorNode>());
  rclcpp::shutdown();
  return 0;
}