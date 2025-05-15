#include <rclcpp/rclcpp.hpp>
#include <holoocean_interfaces/msg/control_command.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <string>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20
#define KEYCODE_DOT 0x2E

class TeleopCommand : public rclcpp::Node
{
public:
  TeleopCommand();
  void keyLoop();

private:
  void timerCallback();

  double fin1_, fin2_, fin3_, max_fin_value_; 
  int thruster_value_;
  std::string vehicle_name_;
  holoocean_interfaces::msg::ControlCommand last_command_msg_;
  rclcpp::Publisher<holoocean_interfaces::msg::ControlCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

TeleopCommand::TeleopCommand() :
  Node("teleop_command"),
  fin1_(0.0), fin2_(0.0), fin3_(0.0)
{
  declare_parameter("max_fin_value", 35.0);
  declare_parameter("thruster_value", 800);
  declare_parameter("holoocean_vehicle", "auv0");

  get_parameter("max_fin_value", max_fin_value_);
  get_parameter("thruster_value", thruster_value_);
  get_parameter("holoocean_vehicle", vehicle_name_);

  command_pub_ = create_publisher<holoocean_interfaces::msg::ControlCommand>("holoocean/command/control", 10);

  // Initialize last command message
  last_command_msg_.header.stamp = get_clock()->now();
  last_command_msg_.header.frame_id = vehicle_name_;
  last_command_msg_.cs = {
    fin1_ * M_PI / 180.0,
    fin2_ * M_PI / 180.0,
    fin3_ * M_PI / 180.0,
    static_cast<double>(thruster_value_)
  };

  timer_ = create_wall_timer(
    std::chrono::milliseconds(1500),
    std::bind(&TeleopCommand::timerCallback, this)
  );
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto teleop_command = std::make_shared<TeleopCommand>();

  signal(SIGINT, quit);

  teleop_command->keyLoop();
  
  rclcpp::spin(teleop_command);
  rclcpp::shutdown();
  return 0;
}

void TeleopCommand::keyLoop()
{
  char c;
  bool dirty = false;

  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to control fins");
  puts("Spacebar increases thruster, '.' decreases thruster");

  for(;;)
  {
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    switch(c)
    {
      case KEYCODE_L:
        RCLCPP_DEBUG(get_logger(), "LEFT");
        fin1_ = std::max(-max_fin_value_, fin1_ - 2.0);
        dirty = true;
        break;
      case KEYCODE_R:
        RCLCPP_DEBUG(get_logger(), "RIGHT");
        fin1_ = std::min(max_fin_value_, fin1_ + 2.0);
        dirty = true;
        break;
      case KEYCODE_U:
        RCLCPP_DEBUG(get_logger(), "UP");
        fin2_ = std::min(max_fin_value_, fin2_ + 2.0);
        fin3_ = std::min(max_fin_value_, fin3_ + 2.0);
        dirty = true;
        break;
      case KEYCODE_D:
        RCLCPP_DEBUG(get_logger(), "DOWN");
        fin2_ = std::max(-max_fin_value_, fin2_ - 2.0);
        fin3_ = std::max(-max_fin_value_, fin3_ - 2.0);
        dirty = true;
        break;
      case KEYCODE_SPACE:
        thruster_value_ = std::min(2000, thruster_value_ + 50);
        RCLCPP_INFO(get_logger(), "Increased thruster to %d", thruster_value_);
        dirty = true;
        break;
      case KEYCODE_DOT:
        thruster_value_ = std::max(0, thruster_value_ - 50);
        RCLCPP_INFO(get_logger(), "Decreased thruster to %d", thruster_value_);
        dirty = true;
        break;
    }

    if (dirty)
    {
      last_command_msg_.header.stamp = get_clock()->now();
      last_command_msg_.header.frame_id = vehicle_name_;
      last_command_msg_.cs = {
        -fin1_ * M_PI / 180.0,
         fin2_ * M_PI / 180.0,
        -fin3_ * M_PI / 180.0,
        static_cast<double>(thruster_value_)
      };

      command_pub_->publish(last_command_msg_);
      dirty = false;
    }
  }
}

void TeleopCommand::timerCallback()
{
  last_command_msg_.header.stamp = get_clock()->now();
  command_pub_->publish(last_command_msg_);
}
