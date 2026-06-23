#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <string>

#include <fcntl.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

class TargetKeyboardControlNode : public rclcpp::Node
{
public:
  TargetKeyboardControlNode()
  : Node("target_keyboard_control_node")
  {
    declare_parameter<std::string>("cmd_vel_topic", "/target_uav/cmd_vel_body");
    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<double>("linear_step", 0.1);
    declare_parameter<double>("vertical_step", 0.05);
    declare_parameter<double>("yaw_step", 0.1);
    declare_parameter<double>("max_linear_speed", 1.0);
    declare_parameter<double>("max_vertical_speed", 0.5);
    declare_parameter<double>("max_yaw_rate", 1.0);

    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    publish_rate_hz_ = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    linear_step_ = std::max(0.0, get_parameter("linear_step").as_double());
    vertical_step_ = std::max(0.0, get_parameter("vertical_step").as_double());
    yaw_step_ = std::max(0.0, get_parameter("yaw_step").as_double());
    max_linear_speed_ = std::max(0.0, get_parameter("max_linear_speed").as_double());
    max_vertical_speed_ = std::max(0.0, get_parameter("max_vertical_speed").as_double());
    max_yaw_rate_ = std::max(0.0, get_parameter("max_yaw_rate").as_double());

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    terminal_ready_ = configure_terminal();

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TargetKeyboardControlNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "target_keyboard_control_node started: topic=%s keys=[w/s x, a/d y, r/f z, q/e yaw, space stop, x quit]",
      cmd_vel_topic_.c_str());
    if (!terminal_ready_) {
      RCLCPP_WARN(get_logger(), "stdin is not a terminal; publishing zero command until input is available");
    }
  }

  ~TargetKeyboardControlNode() override
  {
    publish_zero();
    restore_terminal();
  }

private:
  void timer_callback()
  {
    read_available_keys();
    cmd_pub_->publish(cmd_);
  }

  bool configure_terminal()
  {
    if (!isatty(STDIN_FILENO)) {
      return false;
    }

    if (tcgetattr(STDIN_FILENO, &old_tio_) != 0) {
      RCLCPP_WARN(get_logger(), "tcgetattr failed");
      return false;
    }
    old_tio_valid_ = true;

    termios new_tio = old_tio_;
    new_tio.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    new_tio.c_cc[VMIN] = 0;
    new_tio.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_tio) != 0) {
      RCLCPP_WARN(get_logger(), "tcsetattr failed");
      return false;
    }

    old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (old_flags_ < 0) {
      RCLCPP_WARN(get_logger(), "fcntl(F_GETFL) failed");
      return false;
    }
    old_flags_valid_ = true;
    if (fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK) != 0) {
      RCLCPP_WARN(get_logger(), "fcntl(F_SETFL) failed");
      return false;
    }

    return true;
  }

  void restore_terminal()
  {
    if (old_flags_valid_) {
      (void)fcntl(STDIN_FILENO, F_SETFL, old_flags_);
      old_flags_valid_ = false;
    }
    if (old_tio_valid_) {
      (void)tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
      old_tio_valid_ = false;
    }
  }

  void read_available_keys()
  {
    if (!terminal_ready_) {
      return;
    }

    char c = 0;
    while (true) {
      const ssize_t n = read(STDIN_FILENO, &c, 1);
      if (n == 1) {
        handle_key(c);
        continue;
      }
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "keyboard read failed");
      }
      break;
    }
  }

  void handle_key(char raw_key)
  {
    const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(raw_key)));
    bool changed = true;

    switch (key) {
      case 'w':
        cmd_.linear.x += linear_step_;
        break;
      case 's':
        cmd_.linear.x -= linear_step_;
        break;
      case 'a':
        cmd_.linear.y += linear_step_;
        break;
      case 'd':
        cmd_.linear.y -= linear_step_;
        break;
      case 'r':
        cmd_.linear.z -= vertical_step_;
        break;
      case 'f':
        cmd_.linear.z += vertical_step_;
        break;
      case 'q':
        cmd_.angular.z += yaw_step_;
        break;
      case 'e':
        cmd_.angular.z -= yaw_step_;
        break;
      case ' ':
        cmd_ = geometry_msgs::msg::Twist{};
        break;
      case 'x':
        cmd_ = geometry_msgs::msg::Twist{};
        clamp_command();
        log_command("quit");
        cmd_pub_->publish(cmd_);
        rclcpp::shutdown();
        return;
      default:
        changed = false;
        break;
    }

    if (changed) {
      clamp_command();
      log_command(std::string(1, key));
    }
  }

  void clamp_command()
  {
    cmd_.linear.x = std::clamp(cmd_.linear.x, -max_linear_speed_, max_linear_speed_);
    cmd_.linear.y = std::clamp(cmd_.linear.y, -max_linear_speed_, max_linear_speed_);
    cmd_.linear.z = std::clamp(cmd_.linear.z, -max_vertical_speed_, max_vertical_speed_);
    cmd_.angular.z = std::clamp(cmd_.angular.z, -max_yaw_rate_, max_yaw_rate_);
  }

  void log_command(const std::string & key)
  {
    RCLCPP_INFO(
      get_logger(),
      "key=%s cmd: vx=%.2f vy=%.2f vz=%.2f yaw_rate=%.2f",
      key.c_str(),
      cmd_.linear.x,
      cmd_.linear.y,
      cmd_.linear.z,
      cmd_.angular.z);
  }

  void publish_zero()
  {
    if (cmd_pub_) {
      geometry_msgs::msg::Twist zero{};
      cmd_pub_->publish(zero);
    }
  }

  std::string cmd_vel_topic_;
  double publish_rate_hz_{20.0};
  double linear_step_{0.1};
  double vertical_step_{0.05};
  double yaw_step_{0.1};
  double max_linear_speed_{1.0};
  double max_vertical_speed_{0.5};
  double max_yaw_rate_{1.0};

  geometry_msgs::msg::Twist cmd_{};
  bool terminal_ready_{false};
  bool old_tio_valid_{false};
  bool old_flags_valid_{false};
  int old_flags_{0};
  termios old_tio_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetKeyboardControlNode>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
