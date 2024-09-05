#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <can_msgs/msg/frame.hpp>
#include <cmath>

class AutowareToCantx : public rclcpp::Node
{
public:
  AutowareToCantx() : Node("autoware_to_cantx")
  {
    // ROSパラメータの宣言と初期化
    this->declare_parameter<double>("angle_in_degrees", 40.0);
    this->declare_parameter<double>("brake_cmd", 0.3);

    timer_control_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&AutowareToCantx::controlCmdCallback, this)
    );

    timer_actuation_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&AutowareToCantx::actuationCmdCallback, this)
    );

    // CANトピックのパブリッシャーの初期化
    can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/can_test", 10);
  }

private:
  void controlCmdCallback()
  {
    // ROSパラメータの取得
    RCLCPP_INFO(this->get_logger(),"before getting pram");
    double angle_in_degrees = this->get_parameter("angle_in_degrees").as_double();
    RCLCPP_INFO(this->get_logger(),"receved steering_angle");
    can_msgs::msg::Frame can_enable_msg;
    can_enable_msg.header.stamp = this->get_clock()->now();
    can_enable_msg.id = 0x7DF;
    can_enable_msg.is_rtr = false;
    can_enable_msg.is_extended = false;
    can_enable_msg.is_error = false;
    can_enable_msg.dlc = 8;

    can_enable_msg.data[0] = 0x08; // size
    can_enable_msg.data[1] = 0x08; // mode
    can_enable_msg.data[2] = 0x22; // PID for steering angle
    can_enable_msg.data[3] = 0x01;
    can_enable_msg.data[4] = 0x00;

    // 残りのデータバイトをゼロで埋める
    can_enable_msg.data[5] = 0x00;
    can_enable_msg.data[6] = 0x00;
    can_enable_msg.data[7] = 0x00;

    // CANメッセージを送信
    can_pub_->publish(can_enable_msg);

    // CANフレームを作成（ステアリング角度用）
    can_msgs::msg::Frame can_msg;
    can_msg.header.stamp = this->get_clock()->now();
    can_msg.id = 0x7DF;
    can_msg.is_rtr = false;
    can_msg.is_extended = false;
    can_msg.is_error = false;
    can_msg.dlc = 8;

    const double max_angle_degrees = 540.0; // ±1.5回転が最大角度
    const int16_t max_encoded_value = 32767;

    int16_t steering_angle = static_cast<int16_t>((angle_in_degrees / max_angle_degrees) * max_encoded_value);

    can_msg.data[0] = 0x08; // size
    can_msg.data[1] = 0x08; // mode
    can_msg.data[2] = 0x21; // PID for steering angle
    can_msg.data[3] = (steering_angle >> 8) & 0xFF; // High byte of steering angle
    can_msg.data[4] = steering_angle & 0xFF; // Low byte of steering angle

    // 残りのデータバイトをゼロで埋める
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;

    // CANメッセージを送信
    can_pub_->publish(can_msg);
  }

  void actuationCmdCallback()
  {
    // ROSパラメータの取得
    RCLCPP_INFO(this->get_logger(),"before getting pram");
    //double brake_cmd = 0.3;
    double brake_cmd = this->get_parameter("brake_cmd").as_double();
    RCLCPP_INFO(this->get_logger(),"receved brake_force");
    // CANフレームを作成（ブレーキ力用）
    can_msgs::msg::Frame can_msg_brake;
    can_msg_brake.header.stamp = this->get_clock()->now();
    can_msg_brake.id = 0x7DF;
    can_msg_brake.is_rtr = false;
    can_msg_brake.is_extended = false;
    can_msg_brake.is_error = false;
    can_msg_brake.dlc = 8;

    int16_t brake_force = static_cast<int16_t>(brake_cmd * 32767);

    can_msg_brake.data[0] = 0x08; // size
    can_msg_brake.data[1] = 0x08; // mode
    can_msg_brake.data[2] = 0x26; // PID for brake force
    can_msg_brake.data[3] = (brake_force >> 8) & 0xFF; // High byte of brake force
    can_msg_brake.data[4] = brake_force & 0xFF; // Low byte of brake force

    // 残りのデータバイトをゼロで埋める
    can_msg_brake.data[5] = 0x00;
    can_msg_brake.data[6] = 0x00;
    can_msg_brake.data[7] = 0x00;

    // CANメッセージを送信
    can_pub_->publish(can_msg_brake);
  }

  // タイマー
  rclcpp::TimerBase::SharedPtr timer_control_;
  rclcpp::TimerBase::SharedPtr timer_actuation_;

  // パブリッシャー
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutowareToCantx>());
  rclcpp::shutdown();
  return 0;
}
