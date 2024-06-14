import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

"""
0.357015, -0.695551,
0.533665, -0.544118
4.27509, 5.92226
2.28073, 8.95063
-3.29666, 5.04603
"""


class SimplePubSub(Node):

    # init func 
    def __init__(self):
        super().__init__('nav_point_publisher')
        self.timer = self.create_timer(1, self.timer_callback_once)
        self.navigator = BasicNavigator()
        self.route = [
                        [0.533665, -0.544118],
                        [4.27509, 5.92226],
                        [2.28073, 8.95063],
                        [-3.29666, 5.04603]]
        self.points_sent = 0

    def timer_callback_once(self):
        self.get_logger().info('Waiting for navigation to be active.')
        self.navigator.waitUntilNav2Active()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = self.route[self.points_sent][0]
        pose.pose.position.y = self.route[self.points_sent][1]
        pose.pose.orientation.z = 1.0
        pose.pose.orientation.w = 0.0
        for i in range(len(self.route)):
            self.get_logger().info('Sending Point: "%s"' % "[" + str(pose.pose.position.x) + ", " + str(pose.pose.position.y) + "]")
            self.navigator.goToPose(pose)
            self.points_sent += 1
            pose.pose.position.x = self.route[self.points_sent][0]
            pose.pose.position.y = self.route[self.points_sent][1]
            while not self.navigator.isTaskComplete():
                pass
        self.timer.cancel()
            
def main(args=None):
    rclpy.init(args=args)
    simple_pubsub = SimplePubSub()
    rclpy.spin(simple_pubsub)
    simple_pubsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()