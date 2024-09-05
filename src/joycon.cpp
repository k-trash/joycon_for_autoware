#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <cmath>

class JoyConController : public rclcpp::Node
{
public:
  JoyConController() : Node("joycon_controller")
  {
    control_cmd_pub_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", 10);
    actuation_cmd_pub_ = this->create_publisher<tier4_vehicle_msgs::msg::ActuationCommandStamped>("/control/command/actuation_cmd", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoyConController::joyCallback, this, std::placeholders::_1));
    brake = 0.0;
    steering_angle = 0.0;
    target_steering_angle = 0.0;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&JoyConController::updateSteering, this));
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Assuming axes[1] is for brake and axes[0] is for steering
    brake = 1.0 - msg->axes[4];
    target_steering_angle = -msg->axes[0];

    brake = std::max(0.0f, std::min(1.0f, brake));
    target_steering_angle = std::max(-1.0f, std::min(1.0f, target_steering_angle));
  }

  void updateSteering()
  {
    // Smoothly interpolate towards the target steering angle
    float steering_speed = 0.1; // Adjust this value to control the speed of steering changes
    steering_angle += (target_steering_angle - steering_angle) * steering_speed;

    auto control_msg = autoware_auto_control_msgs::msg::AckermannControlCommand();
    control_msg.lateral.steering_tire_angle = steering_angle;

    auto actuation_msg = tier4_vehicle_msgs::msg::ActuationCommandStamped();
    actuation_msg.actuation.brake_cmd = brake;

    control_cmd_pub_->publish(control_msg);
    actuation_cmd_pub_->publish(actuation_msg);

    RCLCPP_INFO(this->get_logger(), "Brake: %.2f, Steering Angle: %.2f", brake, steering_angle);
  }

  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationCommandStamped>::SharedPtr actuation_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  float brake;
  float steering_angle;
  float target_steering_angle;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyConController>());
  rclcpp::shutdown();
  return 0;
}