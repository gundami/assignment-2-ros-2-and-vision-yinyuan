import rclpy
from rclpy.node import Node
from aisd_msgs.msg import Hand
from geometry_msgs.msg import Twist

class MoveNode(Node):
    def __init__(self):
        super().__init__('move_node')
        
        self.subscription = self.create_subscription(
            Hand,
            'cmd_hand',
            self.listener_callback,
            10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def listener_callback(self, msg):
        twist = Twist()
        
        hand_width = abs(msg.xindex - msg.xpinky)
        hand_center = (msg.xindex + msg.xpinky) / 2.0
        
        twist.linear.x = hand_width * 2.0
        twist.angular.z = (0.5 - hand_center) * 4.0
        
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    move_node = MoveNode()
    rclpy.spin(move_node)
    move_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()