#include <chrono>
#include <memory>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp" // Better than a raw Int32 for distance

using namespace std::chrono_literals;

class UltrasonicSensorNode : public rclcpp::Node {
public:
  UltrasonicSensorNode() : Node("ultrasonic_sensor_node") {
    // 1. Setup Serial Port (Jetson Nano Port 1 is usually /dev/ttyTHS1)
    serial_port_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    setup_serial();
    std::cout << "Serial port initialized for Ultrasonic Sensor" << std::endl;

    // 2. Setup Publisher (Using Range message for ROS 2 standards)
    publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/ultrasonic/distance", 10);

    // 3. Setup Timer (Replaces the Arduino loop)
    timer_ = this->create_wall_timer(100ms, std::bind(&UltrasonicSensorNode::read_sensor, this));
  }

  ~UltrasonicSensorNode() {
    if (serial_port_ >= 0) close(serial_port_);
  }

private:
  void setup_serial() {
    struct termios tty;
    tcgetattr(serial_port_, &tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);    // Ignore modem lines, enable receiver
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                 // 8 bit chars
    tty.c_cflag &= ~PARENB;            // no parity
    tty.c_cflag &= ~CSTOPB;            // 1 stop bit
    tcsetattr(serial_port_, TCSANOW, &tty);
  }

  void read_sensor() {
    uint8_t COM = 0x55;
    write(serial_port_, &COM, 1); // Trigger the sensor

    // Give the sensor a moment to respond
    std::this_thread::sleep_for(10ms);

    uint8_t buffer[4];
    if (read(serial_port_, buffer, 4) == 4) {
      if (buffer[0] == 0xFF) {
        uint8_t checksum = buffer[0] + buffer[1] + buffer[2];
        
        if (buffer[3] == checksum) {
          int distance_mm = (buffer[1] << 8) + buffer[2];
          
          auto message = sensor_msgs::msg::Range();
          message.header.stamp = this->now();
          message.header.frame_id = "ultrasonic_link";
          message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
          message.range = static_cast<float>(distance_mm) / 1000.0f; // Convert to Meters
          
          RCLCPP_INFO(this->get_logger(), "Distance: %d mm", distance_mm);
          publisher_->publish(message);
        }
      }
    }
  }

  int serial_port_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicSensorNode>());
  rclcpp::shutdown();
  return 0;
}