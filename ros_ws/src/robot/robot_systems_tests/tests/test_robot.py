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
import mavros_msgs.srv

import time


@pytest.mark.rostest
# this is the test descriptioon used to launch the full robot system with launch_robot_headless.yaml
def generate_test_description():
    robot_launch_path = '/root/AirStack/ros_ws/src/robot/autonomy/planning/global/central/launch/ascent_drone_headless.xml'

    gui_arg = launch.actions.DeclareLaunchArgument('use_gui', default_value='false', description='Whether to launch the GUI')

    robot_launch = launch.actions.IncludeLaunchDescription( launch.launch_description_sources.AnyLaunchDescriptionSource(robot_launch_path),
                                                            launch_arguments={'use_gui': launch.substitutions.LaunchConfiguration('use_gui')}.items())
    return (
        launch.LaunchDescription([
            gui_arg,
            robot_launch,
            launch_testing.actions.ReadyToTest(),
        ]),
        {}
    )


class TestRobotSystem(unittest.TestCase):
    
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
        self.node = rclpy.create_node('robot_tester_node')
        self.service_timeout = 2

    def tearDown(self):
        self.node.destroy_node()

    def test_set_mode(self):
        client = self.node.create_client(mavros_msgs.srv.SetMode, '/mavros/set_mode')
        self.node.get_logger().info("Waiting for service to be available...")
        accum_time = 0
        while not client.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
            accum_time += 1
            if accum_time > self.service_timeout:
                print('service not available, aborting test...')
                self.assertTrue(False)
        request = mavros_msgs.srv.SetMode.Request()
        request.custom_mode = "GUIDED"
        print("Sending request to set mode to GUIDED") 
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        self.assertTrue(response.mode_sent)


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




        
        

