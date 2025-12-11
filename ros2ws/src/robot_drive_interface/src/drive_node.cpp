#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/motor_control.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <stdexcept>

#include "shared_defs/RobotIO.h"

using std::placeholders::_1;

class RobotDriveInterface final : public rclcpp::Node
{
public:
  RobotDriveInterface()
  : Node("robot_drive_interface")
  {
    // Declare parameter with default value
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    std::string port = this->get_parameter("serial_port").as_string();

    init_interface(port);

    subscription_ = this->create_subscription<robot_msgs::msg::MotorControl>(
      "command_node", 10,
      std::bind(&RobotDriveInterface::topic_callback, this, _1));
  }

private:

  void init_interface(const std::string& device)
  {
    tty_fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);

    if (tty_fd_ < 0) {
      throw std::runtime_error("Failed to open serial port: " + device);
    }

    termios tio{};
    tcgetattr(tty_fd_, &tio);

    tio.c_cflag = CS8 | CREAD | CLOCAL;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;

    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);

    tcflush(tty_fd_, TCIFLUSH);
    tcsetattr(tty_fd_, TCSANOW, &tio);

    RCLCPP_INFO(this->get_logger(), "Opened serial port: %s", device.c_str());
  }

  void topic_callback(const robot_msgs::msg::MotorControl & msg) const
  {
    RCLCPP_INFO(this->get_logger(),
                "Received motor command: left=%d right=%d",
                msg.left_motor, msg.right_motor);

    shared_defs::MotorCommandUnion u{};
    u.motor_command.left_motor = msg.left_motor;
    u.motor_command.right_motor = msg.right_motor;

    write(tty_fd_, &shared_defs::sync_byte, 1);
    write(tty_fd_, u.raw_data, sizeof(u.raw_data));
  }

  int tty_fd_{};
  rclcpp::Subscription<robot_msgs::msg::MotorControl>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDriveInterface>());
  rclcpp::shutdown();
  return 0;
}
