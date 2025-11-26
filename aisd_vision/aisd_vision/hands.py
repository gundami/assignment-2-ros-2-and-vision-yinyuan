import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
mp_hands = mp.solutions.hands

from aisd_msgs.msg import Hand

class HandDetectionNode(Node):
    def __init__(self):
        super().__init__('hand_detection_node')
        
        # Create subscription to camera images
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        
        # A converter between ROS and OpenCV images
        self.br = CvBridge()
        
        # Create publisher for Hand messages to cmd_hand topic
        self.hand_publisher = self.create_publisher(Hand, 'cmd_hand', 10)
        
        self.get_logger().info('Hand detection node started')
    
    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        image = self.br.imgmsg_to_cv2(msg)
        
        # Define landmark indices for finger tips
        PINKY_FINGER_TIP = 20
        INDEX_FINGER_TIP = 8
        
        # Analyze the image for hands using MediaPipe
        with mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as myhands:
            
            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            # Convert image from BGR to RGB for MediaPipe processing
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # Process the image to detect hands
            results = myhands.process(image)
            
            # If hand landmarks are detected
            if results.multi_hand_landmarks:
                # Publish the hand position in terms of index finger and pinky
                msg = Hand()
                # Get x-coordinate of pinky finger tip
                msg.xpinky = results.multi_hand_landmarks[0].landmark[PINKY_FINGER_TIP].x
                # Get x-coordinate of index finger tip
                msg.xindex = results.multi_hand_landmarks[0].landmark[INDEX_FINGER_TIP].x
                
                # Only publish if there are subscribers listening
                if self.hand_publisher.get_subscription_count() > 0:
                    self.hand_publisher.publish(msg)
                else:
                    self.get_logger().info('waiting for subscriber')

def main(args=None):
    rclpy.init(args=args)
    hand_detection_node = HandDetectionNode()
    
    try:
        rclpy.spin(hand_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        hand_detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()