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

from std_msgs.msg import String

import time

class End2EndTestNode(Node):
    def __init__(self):
        super().__init__('end2end_test_node')
        self.subscription = self.create_subscription(String, '/sim_status', self.sim_status_callback, 10)
        self.sim_status = None

    def sim_status_callback(self, msg):
        self.sim_status = msg.data
        self.get_logger().info(f'variable is now: {self.sim_status}')

@pytest.mark.rostest
# this is the test descriptioon used to launch the full robot system with launch_robot_headless.yaml
def generate_test_description():
    robot_launch_path = '/root/AirStack/ros_ws/src/end2end_tests/launch/isaac_test_env_bringup.xml'

    robot_launch = launch.actions.IncludeLaunchDescription( launch.launch_description_sources.AnyLaunchDescriptionSource(robot_launch_path))

    return (
        launch.LaunchDescription([
            robot_launch,
            launch_testing.actions.ReadyToTest(),
        ]),
        {}
    )


class TestEnd2End(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = End2EndTestNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_wait_for_sim(self, timeout=30):
        # Wait for the isaac sim to be available
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node)
            if self.node.sim_status == "ready":
                self.assertTrue(True, "Simulation was initialized successfully")
                return
        self.assertTrue(False, "Timeout waiting for isaac sim to be ready")


    # def test_arming(self):
    #     # Create a service call to existing "mavros/cmd/arming" service
    #     client = self.node.create_client(mavros_msgs.srv.CommandBool, '/mavros/cmd/arming')
    #     client.wait_for_service()
    #     request = mavros_msgs.srv.CommandBool.Request()
    #     request.value = True
    #     future = client.call_async(request)
    #     rclpy.spin_until_future_complete(self.node, future)
    #     response = future.result()
    #     self.assertTrue(response.success)

    # def test_disarming(self):
    #     # Create a service call to existing "mavros/cmd/arming" service
    #     client = self.node.create_client(mavros_msgs.srv.CommandBool, '/mavros/cmd/arming')
    #     client.wait_for_service()
    #     request = mavros_msgs.srv.CommandBool.Request()
    #     request.value = False
    #     future = client.call_async(request)
    #     rclpy.spin_until_future_complete(self.node, future)
    #     response = future.result()
    #     self.assertTrue(response.success)




        
        

