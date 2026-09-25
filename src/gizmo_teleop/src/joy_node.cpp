#include "gizmo_teleop/joy_node.hpp"

JoyNode::JoyNode()
: Node("joy_gui_node")
{
  topic_name_ = this->declare_parameter<std::string>("topic_name", "cmd_vel");
  use_twist_stamped_ = this->declare_parameter<bool>("twistStamped", false);
  max_linear_ = this->declare_parameter<double>("max_linear", 1.0);
  max_angular_ = this->declare_parameter<double>("max_angular", 3.14);
  turtle_mode_speed_ = this->declare_parameter<double>("turtle_mode_speed", 0.5);
  rabbit_mode_speed_ = this->declare_parameter<double>("rabbit_mode_speed", 3.0);
  publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);

  if (use_twist_stamped_) {
    publisher_twist_stamped_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(topic_name_, 10);
  } else {
    publisher_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name_, 10);
  }

  double timer_period = (publish_rate_hz_ > 0.0) ? (1.0 / publish_rate_hz_) : 0.1;
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period),
    std::bind(&JoyNode::publishTwist, this));

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&JoyNode::parameterCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult JoyNode::parameterCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (param.get_name() == "topic_name" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      changeTopic(param.as_string());
    } else if (param.get_name() == "twistStamped" && param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      setTwistStamped(param.as_bool());
    } else if (param.get_name() == "max_linear" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      max_linear_ = param.as_double();
      if (gui_update_linear_cb) {
        gui_update_linear_cb(max_linear_);
      }
    } else if (param.get_name() == "max_angular" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      max_angular_ = param.as_double();
      if (gui_update_angular_cb) {
        gui_update_angular_cb(max_angular_);
      }
    } else if (param.get_name() == "publish_rate_hz" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      double new_rate = param.as_double();
      if (new_rate > 0.0) {
        publish_rate_hz_ = new_rate;
        timer_->cancel();
        timer_ = this->create_wall_timer(
          std::chrono::duration<double>(1.0 / publish_rate_hz_),
          std::bind(&JoyNode::publishTwist, this));
        RCLCPP_INFO(this->get_logger(), "Publish rate changed dynamically to: %.1f Hz", publish_rate_hz_);
      }
    } else if (param.get_name() == "turtle_mode_speed" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      turtle_mode_speed_ = param.as_double();
    } else if (param.get_name() == "rabbit_mode_speed" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      rabbit_mode_speed_ = param.as_double();
    }
  }

  return result;
}

void JoyNode::changeTopic(const std::string & new_topic)
{
  if (!new_topic.empty() && new_topic != topic_name_) {
    topic_name_ = new_topic;
    publisher_twist_.reset();
    publisher_twist_stamped_.reset();

    if (use_twist_stamped_) {
      publisher_twist_stamped_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(topic_name_, 10);
    } else {
      publisher_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name_, 10);
    }

    RCLCPP_INFO(this->get_logger(), "Changed publisher topic to: %s", topic_name_.c_str());
    if (gui_update_topic_cb) {
      gui_update_topic_cb(topic_name_);
    }
  }
}

void JoyNode::setTwistStamped(bool enabled)
{
  if (use_twist_stamped_ != enabled) {
    use_twist_stamped_ = enabled;
    publisher_twist_.reset();
    publisher_twist_stamped_.reset();

    if (use_twist_stamped_) {
      publisher_twist_stamped_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(topic_name_, 10);
      RCLCPP_INFO(this->get_logger(), "Switched publisher mode to: TwistStamped");
    } else {
      publisher_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name_, 10);
      RCLCPP_INFO(this->get_logger(), "Switched publisher mode to: Twist");
    }

    if (gui_update_twist_stamped_cb) {
      gui_update_twist_stamped_cb(use_twist_stamped_);
    }
  }
}

void JoyNode::updateTwist(double norm_x, double norm_y)
{
  double linear_mult = invert_linear_ ? -1.0 : 1.0;
  double angular_mult = invert_angular_ ? -1.0 : 1.0;

  double lin_x = norm_y * max_linear_ * linear_mult;
  double ang_z = -norm_x * max_angular_ * angular_mult;

  twist_msg_.linear.x = lin_x;
  twist_msg_.angular.z = ang_z;

  twist_stamped_msg_.twist.linear.x = lin_x;
  twist_stamped_msg_.twist.angular.z = ang_z;
}

void JoyNode::publishTwist()
{
  if (use_twist_stamped_) {
    if (publisher_twist_stamped_) {
      twist_stamped_msg_.header.stamp = this->now();
      twist_stamped_msg_.header.frame_id = "base_footprint";
      publisher_twist_stamped_->publish(twist_stamped_msg_);
    }
  } else {
    if (publisher_twist_) {
      publisher_twist_->publish(twist_msg_);
    }
  }
}
