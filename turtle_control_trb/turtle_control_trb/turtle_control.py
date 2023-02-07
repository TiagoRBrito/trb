import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import math

class TurtleControl(Node):
    def __init__(self):
        super().__init__('turtle_control')
        self.init_variables()
        self.init_subscribers()
        self.init_publisher()

    def init_variables(self):
        self.x = 0.0
        self.y = 0.0
        self.x_error = 0.0
        self.y_error = 0.0
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.alpha = 0.0
        self.theta = 0.0
        self.k_omega = 1.0
        self.v_max = 1.0
        self.p = 0.0

    def init_subscribers(self):
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.subscription = self.create_subscription(Pose2D, 'goal', self.goal_callback, 10)

        # Previne aviso de variável sem uso
        self.subscription

    def init_publisher(self):
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.5  # segundos
        self.timer = self.create_timer(timer_period, self.pub_callback)  

    # Recebe pose do turtlesim;
    def pose_callback(self, message):
        self.x = message.x
        self.y = message.y
        self.theta = message.theta

    # Recebe posição desejada;
    def goal_callback(self, message):
        self.x_goal = message.x
        self.y_goal = message.y

    # Método principal do nó.
    def pub_callback(self):
        self.x_error = self.x_goal - self.x
        self.y_error = self.y_goal - self.y
        
        if((abs(self.x_error) > 0.1) or (abs(self.y_error) > 0.1)):
            self.alpha = math.atan2(self.y_error,self.x_error) - (self.theta)
            self.p = math.sqrt((self.x_error**2) + (self.y_error**2))
        else:
            self.alpha = 0.0
            self.p = 0.0

        message = Twist()
        message.linear.x = math.tanh(self.p) * self.v_max
        message.linear.y = 0.0
        message.angular.z = self.alpha * self.k_omega

        self.publisher_.publish(message)

def main(args=None):
    rclpy.init(args=args)
    turtle_control = TurtleControl()
    rclpy.spin(turtle_control)
    turtle_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
