#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from aisd_msgs.srv import Speak


class SpeakerClient(Node):
    """
    ROS2 node that listens to the /words topic and speaks the recognized text
    by calling the /speak service.
    """

    def __init__(self):
        super().__init__('speak_client')
        
        # Create service client for /speak
        self.speak_client = self.create_client(Speak, 'speak')
        
        # Wait for the /speak service to be available
        self.get_logger().info('Waiting for /speak service...')
        while not self.speak_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/speak service not available, waiting...')
        
        self.get_logger().info('/speak service is ready')
        
        # Subscribe to /words topic
        self.subscription = self.create_subscription(
            String,
            'words',
            self.words_callback,
            10)
        
        self.get_logger().info('Speaker client ready and listening to /words topic')

    def words_callback(self, msg):
        """
        Callback function triggered when a message is received on /words topic.
        
        Args:
            msg: String message containing recognized text
        """
        self.get_logger().info(f'Received words: "{msg.data}"')
        
        # Skip empty or whitespace-only messages
        if not msg.data or not msg.data.strip():
            self.get_logger().warn('Received empty text, skipping...')
            return
        
        # Call the speak service
        self.call_speak_service(msg.data)

    def call_speak_service(self, text):
        """
        Call the /speak service to convert text to speech.
        
        Args:
            text: String to be spoken
        """
        # Create service request
        request = Speak.Request()
        request.words = text
        
        self.get_logger().info(f'Calling /speak service: "{text}"')
        
        # Call service asynchronously
        future = self.speak_client.call_async(request)
        future.add_done_callback(self.speak_response_callback)

    def speak_response_callback(self, future):
        """
        Handle the response from the /speak service.
        
        Args:
            future: Future object containing the service response
        """
        try:
            response = future.result()
            self.get_logger().info(f'Speak service response: {response.response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    """
    Main function to initialize and run the speaker client node.
    """
    rclpy.init(args=args)
    
    speaker_client = SpeakerClient()
    
    try:
        rclpy.spin(speaker_client)
    except KeyboardInterrupt:
        pass
    finally:
        speaker_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()