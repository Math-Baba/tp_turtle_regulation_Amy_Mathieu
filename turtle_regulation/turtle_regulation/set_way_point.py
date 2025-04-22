import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from math import atan2, atan, tan, pi, sqrt
from turtle_interfaces.srv import SetWayPoint
 
class Publisher(Node):
    def __init__(self):
        super().__init__('move_towards_waypoint')
 
        self.current_pose = None
        self.waypoint = [5.0, 5.0]
 
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',  
            self.update_pose,
            10)
 
        self.position_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10)
        
        self.is_moving_publisher = self.create_publisher(
            Bool,
            'is_moving',
            10)
        
        self.service = self.create_service(
            SetWayPoint,
            'set_way_point_service',
            self.set_waypoint_callback
            )
 
        self.timer = self.create_timer(0.1, self.publisher_callback)
 
    def update_pose(self, msg):
        self.current_pose = msg
        x = msg.x
        y = msg.y  
        z = msg.theta
        
 
    def publisher_callback(self):
        if self.current_pose is None:
            return
 
        x = self.current_pose.x
        #self.get_logger().info(f"Pose x : {x}")
        y = self.current_pose.y
        #self.get_logger().info(f"Pose y : {y}")
        theta = self.current_pose.theta
 
        angle = atan2(self.waypoint[1] - self.current_pose.y, self.waypoint[0] - self.current_pose.x)
        #self.get_logger().info(f"Angle : {angle}")
 
        e = atan(tan(angle - self.current_pose.theta)/2)
        #self.get_logger().info(f"Erreur angulaire :{e}")
 
        e_l = sqrt((self.waypoint[1] - y) ** 2 + (self.waypoint[0] - x) ** 2)
        #self.get_logger().info(f"Erreur linéaire : {e_l}")
        
        distance_tolerance = 2.0
 
        if e_l < distance_tolerance:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.position_publisher.publish(stop_msg)

            msg = Bool()
            msg.data = False
            self.is_moving_publisher.publish(msg)
            return
        else :
            msg = Bool()
            msg.data = True
            self.is_moving_publisher.publish(msg)
            
           
        msg = Twist()
        Kpl = 0.3
        Kp = 0.8
        msg.angular.z = Kp * e
        msg.linear.x = Kpl * e_l
        self.position_publisher.publish(msg)
 
    def set_waypoint_callback(self, request, response):
        self.waypoint[0] = request.x
        self.waypoint[1] = request.y
        self.get_logger().info(f"Reçu waypoint: ({request.x}, {request.y})")
        response.res = True
        return response
        
 
        
def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()