import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

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