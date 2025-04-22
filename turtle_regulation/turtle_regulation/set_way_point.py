import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from math import sqrt, pi, atan2, tan

class BasicSubscriber(Node):
    def __init__(self):
        super().__init__('set_way_point')
        self.current_pose = None
        self.waypoint = [7.0, 7.0] 
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.update_pose,
            10)
       
        

    def update_pose(self, msg):
        self.current_pose = msg
        x = msg.x
        y = msg.y   
        angle = atan2(self.waypoint[1] - self.current_pose.y, self.waypoint[0] - self.current_pose.x)
        self.get_logger().info(
            f'Position actuelle : x={x:.2f}, y={y:.2f}, theta={msg.theta:.2f} | angle vers waypoint = {angle:.2f} rad'
        )

def main():
    rclpy.init()
    node = BasicSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()