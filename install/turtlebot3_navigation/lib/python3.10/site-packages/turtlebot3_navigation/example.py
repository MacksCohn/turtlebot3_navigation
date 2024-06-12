import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

"""
Position(0.533665, -0.544118, 0), Orientation(0, 0, 0.816279, 0.577657) = Angle: 1.90988

Position(0.533665, -0.544118, 0), Orientation(0, 0, 0.816279, 0.577657) = Angle: 1.90988

Position(4.27509, 5.92226, 0), Orientation(0, 0, 0.874775, 0.484529) = Angle: 2.12994

Position(2.28073, 8.95063, 0), Orientation(0, 0, -0.951347, 0.308121) = Angle: -2.51516

Position(-3.29666, 5.04603, 0), Orientation(0, 0, -0.492306, 0.870422) = Angle: -1.02947
"""

class SimplePubSub(Node):

    # init func
    def __init__(self):
        super().__init__('simple_pubsub')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # is the most used to read LaserScan data and some sensor data.

    def listener_callback(self, msg):
        self.get_logger().info('I receive: "%s"' % str(msg))

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
            
def main(args=None):
    rclpy.init(args=args)
    simple_pubsub = SimplePubSub()
    rclpy.spin(simple_pubsub)
    simple_pubsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()