#ifndef GIZMO_TELEOP__JOY_NODE_HPP_
#define GIZMO_TELEOP__JOY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <string>
#include <functional>
#include <vector>

class JoyNode : public rclcpp::Node
{
public:
  JoyNode();

  void changeTopic(const std::string & new_topic);
  void setTwistStamped(bool enabled);
  void updateTwist(double norm_x, double norm_y);
  void publishTwist();

  // Callbacks for GUI synchronization
  std::function<void(const std::string &)> gui_update_topic_cb;
  std::function<void(double)> gui_update_linear_cb;
  std::function<void(double)> gui_update_angular_cb;
  std::function<void(bool)> gui_update_twist_stamped_cb;

  std::string topic_name_{"cmd_vel"};
  bool use_twist_stamped_{false};

  geometry_msgs::msg::Twist twist_msg_{};
  geometry_msgs::msg::TwistStamped twist_stamped_msg_{};

  bool invert_linear_{false};
  bool invert_angular_{false};

  double max_linear_{1.0};
  double max_angular_{3.14};
  double turtle_mode_speed_{0.5};
  double rabbit_mode_speed_{3.0};
  double publish_rate_hz_{10.0};

private:
  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_twist_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_twist_stamped_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif  // GIZMO_TELEOP__JOY_NODE_HPP_
