import rclpy
from rclpy.node import Node
from turtle_interfaces.srv import SetWayPoint
from std_msgs.msg import Bool


class TurtleClient(Node):
    def __init__(self):
        super().__init__('turtle_client')
        self.set_waypoint_srv=self.create_client(SetWayPoint, 'set_way_point_service')
        
        while not self.set_waypoint_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service indisponible, en attente...')

        self.request=SetWayPoint.Request()

        self.subscription = self.create_subscription(
            Bool,
            'is_moving',
            self.is_moving_callback,
            10
        )
 
        self.is_moving = True
        self.goal_index = 0
 
        self.goals = [
            (7.0, 2.0),
            (7.0, 7.0),
            (2.0, 7.0),
            (2.0, 2.0)
        ]
 
    def is_moving_callback(self, msg):
        self.is_moving = msg.data
 
        if not self.is_moving and self.goal_index < len(self.goals):
            x, y = self.goals[self.goal_index]
            self.send_request(x, y)
            self.goal_index += 1

    def send_request(self, x, y):
        if not self.is_moving:
            self.request.x=x
            self.request.y=y
            self.get_logger().info(f"Envoi du waypoint : ({x}, {y})")


            future=self.set_waypoint_srv.call_async(self.request)
            rclpy.spin_until_future_complete(self, future)

            return future.result() 
        else:
            self.get_logger().info('Tortue en mouvement')     
            return None

def main():
    rclpy.init()
    node=TurtleClient()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__=='__main__':
    main()