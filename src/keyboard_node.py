import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from tier4_vehicle_msgs.msg import ActuationCommandStamped
import sys
import termios
import tty

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.control_cmd_pub_ = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', 10)
        self.actuation_cmd_pub_ = self.create_publisher(ActuationCommandStamped, '/control/command/actuation_cmd', 10)
        self.brake = 0.0
        self.steering_angle = 0.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.configure_terminal()

    def configure_terminal(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def reset_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        return sys.stdin.read(1)

    def timer_callback(self):
        key = self.get_key()
        if key == 'w':
            self.brake += 0.1
        elif key == 's':
            self.brake -= 0.1
        elif key == 'd':
            self.steering_angle += 0.05
        elif key == 'a':
            self.steering_angle -= 0.05
        elif key == 'i':
            self.brake = 0.0
            self.steering_angle = 0.0

        self.brake = max(0.0, min(1.0, self.brake))
        self.steering_angle = max(-1.0, min(1.0, self.steering_angle))

        control_msg = AckermannControlCommand()
        control_msg.lateral.steering_tire_angle = self.steering_angle

        actuation_msg = ActuationCommandStamped()
        actuation_msg.actuation.brake_cmd = self.brake

        self.control_cmd_pub_.publish(control_msg)
        self.actuation_cmd_pub_.publish(actuation_msg)
        self.get_logger().info(f'Brake: {self.brake:.2f}, Steering Angle: {self.steering_angle:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reset_terminal()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()