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
  : Node("minimal_subscriber")
  {
    init_interface();

    subscription_ = this->create_subscription<robot_msgs::msg::MotorControl>(
      "command_node", 10,
      std::bind(&RobotDriveInterface::topic_callback, this, _1)); // NOLINT(*-avoid-bind)
  }

private:

  void init_interface() {
    tty_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (tty_fd_ < 0) {
      throw std::runtime_error("Failed to open serial port");
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
  }

  void topic_callback(const robot_msgs::msg::MotorControl & msg) const {
    // Log message
    RCLCPP_INFO(this->get_logger(), "I heard: '%d' and '%d'",
                msg.left_motor, msg.right_motor);

    // Build packet
    shared_defs::MotorCommandUnion u{};
    u.motor_command.left_motor = msg.left_motor;
    u.motor_command.right_motor = msg.right_motor;

    // Write sync byte + payload
    write(tty_fd_, &shared_defs::sync_byte, 1);
    write(tty_fd_, u.raw_data, sizeof(u.raw_data));
  }

  int tty_fd_{};
  rclcpp::Subscription<robot_msgs::msg::MotorControl>::SharedPtr subscription_;
};

int main(const int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDriveInterface>());
  rclcpp::shutdown();
  return 0;
}
