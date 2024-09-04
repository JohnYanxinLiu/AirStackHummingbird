import os
import sys
import time
import unittest
import uuid

import launch
from launch.launch_service import LaunchService
import launch_ros
import launch_ros.actions
import launch_testing.actions
from launch_testing.io_handler import ActiveIoHandler
import launch_testing_ros

import pytest

import rclpy
from rclpy.node import Node

import std_msgs.msg
from std_msgs.msg import String

import time


@pytest.mark.rostest
# this is the test descriptioon used to launch the full robot system with launch_robot_headless.yaml
def generate_test_description():
    path_to_test = os.path.dirname(__file__)
    robot_launch_path = '/root/AirStack/ros_ws/src/robot/robot_bringup/launch/launch_robot_headless.yaml'
    print("Hello I am starting")

    return (
        launch.LaunchDescription([
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(robot_launch_path)
            ),
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'robot_launch_path': robot_launch_path,
        }
    )


class TestTalkerListenerLink(unittest.TestCase):

    # @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('robot_tester_node')

    def tearDown(self):
        self.node.destroy_node()

    # def test_talker_listener_link(self):
    #     # Create a subscriber to listen to the talker
    #     self.subscriber = self.node.create_subscription(
    #         String, '/chatter', self.listener_callback, 10)

    #     # Create a publisher to talk
    #     self.publisher = self.node.create_publisher(String, '/chatter', 10)

    #     # Publish a message
    #     msg = String()
    #     msg.data = 'Hello, World!'
    #     self.publisher.publish(msg)

    #     # Wait for the message to be received
    #     time.sleep(1)

    #     # Check that the message was received
    #     self.assertTrue(self.received_message)
        

