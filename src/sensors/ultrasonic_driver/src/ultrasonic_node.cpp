#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <vector>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;

class UltrasonicSensorNode : public rclcpp::Node
{
public:
  UltrasonicSensorNode() : Node("ultrasonic_sensor_node"), serial_port_(-1), timer_(nullptr)
  {
    // --- 1. DECLARE PARAMETERS ---
    // Using basic descriptors without ranges for simple text/numeric inputs
    auto device_desc = rcl_interfaces::msg::ParameterDescriptor{};
    device_desc.description = "The serial port device path (e.g., /dev/ttyUSB0)";
    this->declare_parameter<std::string>("serial_device", "/dev/ttyUSB0", device_desc);

    auto freq_desc = rcl_interfaces::msg::ParameterDescriptor{};
    freq_desc.description = "Frequency to poll the sensor in Hz";
    this->declare_parameter<double>("publish_frequency_hz", 10.0, freq_desc);

    // --- 2. INITIALIZE STATE ---
    serial_device_ = this->get_parameter("serial_device").as_string();
    publish_frequency_hz_ = this->get_parameter("publish_frequency_hz").as_double();

    publisher_ = this->create_publisher<std_msgs::msg::Float32>("ultrasonic/distance", 10);

    diagnostics_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    // Initial setup
    open_serial_port();
    update_timer();

    // --- 3. REGISTER DYNAMIC CALLBACK ---
    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&UltrasonicSensorNode::on_set_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Node started. Parameters are now dynamic.");
  }

  ~UltrasonicSensorNode()
  {
    close_serial_port();
  }

private:
  rcl_interfaces::msg::SetParametersResult on_set_parameters(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto &param : parameters)
    {
      if (param.get_name() == "serial_device")
      {
        serial_device_ = param.as_string();
        RCLCPP_INFO(this->get_logger(), "Updating serial_device to: %s", serial_device_.c_str());
        open_serial_port();
      }
      else if (param.get_name() == "publish_frequency_hz")
      {
        double val = param.as_double();
        if (val <= 0.0)
        {
          result.successful = false;
          result.reason = "Frequency must be > 0";
        }
        else
        {
          publish_frequency_hz_ = val;
          update_timer();
          RCLCPP_INFO(this->get_logger(), "Frequency updated to %.2f Hz", val);
        }
      }
    }
    return result;
  }

  void open_serial_port()
  {
    close_serial_port();
    serial_port_ = open(serial_device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not open %s: %s", serial_device_.c_str(), std::strerror(errno));
      return;
    }

    struct termios tty;
    if (tcgetattr(serial_port_, &tty) == 0)
    {
      cfsetospeed(&tty, B115200);
      cfsetispeed(&tty, B115200);
      tty.c_cflag |= (CLOCAL | CREAD);
      tty.c_cflag &= ~PARENB;
      tty.c_cflag &= ~CSTOPB;
      tty.c_cflag &= ~CSIZE;
      tty.c_cflag |= CS8;
      tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
      tty.c_iflag &= ~(IXON | IXOFF | IXANY);
      tty.c_oflag &= ~OPOST;
      tty.c_cc[VMIN] = 0;
      tty.c_cc[VTIME] = 1;
      tcsetattr(serial_port_, TCSANOW, &tty);
    }
    tcflush(serial_port_, TCIOFLUSH);
  }

  void close_serial_port()
  {
    if (serial_port_ >= 0)
    {
      close(serial_port_);
      serial_port_ = -1;
    }
  }

  void update_timer()
  {
    if (timer_)
      timer_->cancel();
    auto period = std::chrono::duration<double>(1.0 / publish_frequency_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&UltrasonicSensorNode::read_sensor, this));
  }

  void read_sensor()
  {
    float distance_m = -1.0f; // Declared ONCE

    if (serial_port_ >= 0) {
      uint8_t trigger_byte = 0x55;
      if (write(serial_port_, &trigger_byte, 1) >= 0) {
        std::this_thread::sleep_for(70ms);

        uint8_t header = 0;
        bool found_header = false;
        for (int i = 0; i < 32; ++i) { 
          if (read(serial_port_, &header, 1) > 0 && header == 0xFF) {
            found_header = true;
            break;
          }
        }

        if (found_header) {
          uint8_t data[3]; 
          if (read(serial_port_, data, 3) == 3) {
            uint8_t high = data[0];
            uint8_t low  = data[1];
            uint8_t sum  = data[2];

            if (((0xFF + high + low) & 0xFF) == sum) {
              // Update the outer variable (no 'float' keyword here)
              distance_m = static_cast<float>((high << 8) | low) / 1000.0f;
              
              auto msg = std_msgs::msg::Float32();
              msg.data = distance_m;
              publisher_->publish(msg);
              
              RCLCPP_INFO(this->get_logger(), "Distance: %.3f m", distance_m);
            }
          }
        }
      }
    }
      
    // --- DIAGNOSTICS ---
    auto diag_msg = diagnostic_msgs::msg::DiagnosticArray();
    rclcpp::Time now = this->get_clock()->now();
    diag_msg.header.stamp.sec = static_cast<std::int32_t>(now.seconds());
    diag_msg.header.stamp.nanosec = static_cast<std::uint32_t>(now.nanoseconds() % 1000000000);

    auto status = diagnostic_msgs::msg::DiagnosticStatus();
    status.name = this->get_name();
    
    auto kv = diagnostic_msgs::msg::KeyValue();
    kv.key = "distance";

    if (distance_m < 0.0f) { 
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Sensor not connected / no data";
      kv.value = "N/A";
    } else {
      kv.value = std::to_string(distance_m);
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "OK";

      if (distance_m < 0.03f) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Not publishing valid data";
      } else if (distance_m < 1.0f) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = std::string("Obstacle very close: ") + std::to_string(distance_m) + "m, check collision avoidance";
      } else if (distance_m < 1.5f) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = std::string("Approaching obstacle: ") + std::to_string(distance_m) + "m";
      }
    }

    status.values = {kv};
    diag_msg.status.push_back(status);
    diagnostics_publisher_->publish(diag_msg);
  }

  int serial_port_;
  std::string serial_device_;
  double publish_frequency_hz_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicSensorNode>());
  rclcpp::shutdown();
  return 0;
}