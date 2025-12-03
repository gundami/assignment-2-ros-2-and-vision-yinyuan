#!/usr/bin/env python

from std_msgs.msg import String
from aisd_msgs.srv import Speak
import rclpy
from rclpy.node import Node

import wave
import struct

import sys
import glob
import os

import whisper

class WordsPublisher(Node):
    def __init__(self):
        super().__init__('words_publisher')
        self.publisher_ = self.create_publisher(String, 'words', 10)
        self.subscription = self.create_subscription(
            String,
            'recording',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.model = whisper.load_model("base")

    def listener_callback(self, msg):
        self.get_logger().info('whisper is analysing: "%s"' % msg.data)
        audio_file = wave.open(msg.data)
        fs = audio_file.getframerate()
        audio_string = audio_file.readframes(-1)
        audio = [struct.unpack("<h", audio_string[i:i+2])[0]
                 for i in range(0, len(audio_string), 2)]

        text = String()
        text.data = self.mystt(fs, msg.data)
        self.get_logger().info('I heard: "%s"' % text.data)
        self.publisher_.publish(text)

    def mystt(self,fs, audio):
        transcription = self.model.transcribe(audio,fp16=False, language='English')
        self.get_logger().info('transcribe_result is: "%s"' % transcription['text'])
        return transcription['text'].strip()

def main(args=None):
    """Standard ROS2 node initialization and spinning"""
    rclpy.init(args=args)
    words_publisher = WordsPublisher()
    rclpy.spin(words_publisher)
    words_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
