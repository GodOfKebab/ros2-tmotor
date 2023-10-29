import rclpy
from rclpy.node import Node

from custom_messages.msg import TmotorMotorControlCommand
from custom_messages.msg import TmotorMotorState
from sensor_msgs.msg import Joy


class JoyToMITConverter(Node):

    def __init__(self):
        super().__init__('joy_to_mit')
        self.theta = 0.
        self.theta_sum_err = 0.
        self.theta_injection_rate = 0.05
        self.publisher_ = self.create_publisher(TmotorMotorControlCommand, '/micro_ros_teensy/set_motor_control', 10)
        timer_freq = 100.  # seconds
        self.timer = self.create_timer(1./timer_freq, self.timer_callback)

        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        self.tmotor_state_subscription = self.create_subscription(
            TmotorMotorState,
            '/micro_ros_teensy/motor_state',
            self.tmotor_state_callback,
            10)

    def timer_callback(self):
        tmotor_msg = TmotorMotorControlCommand()
        tmotor_msg.angular_position = self.theta
        tmotor_msg.k_p = 2.
        tmotor_msg.angular_velocity = 0.
        tmotor_msg.k_d = 0.5
        tmotor_msg.torque =  0.
        self.publisher_.publish(tmotor_msg)

    def tmotor_state_callback(self, msg):
        self.theta_sum_err += self.theta - msg.angular_position

    def joy_callback(self, msg):
        self.theta = (1. - self.theta_injection_rate) * self.theta + self.theta_injection_rate * msg.axes[2]

def main(args=None):
    rclpy.init(args=args)

    joyToMITConverter = JoyToMITConverter()

    rclpy.spin(joyToMITConverter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joyToMITConverter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()