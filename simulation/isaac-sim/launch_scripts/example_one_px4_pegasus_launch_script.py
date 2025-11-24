#!/usr/bin/env python
"""
Minimal PegasusApp launcher that:
 - Starts Isaac Sim
 - Enables required extensions
 - Creates a Pegasus world
 - Starts the timeline and steps until closed
"""

import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment (Must start this before importing omni modules)
simulation_app = SimulationApp({"headless": False})

import omni.kit.app
import omni.timeline
from omni.isaac.core.world import World

# Pegasus imports
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_ouster_lidar import add_ouster_lidar_subgraph
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from scipy.spatial.transform import Rotation
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
import numpy as np
import os
import subprocess
import threading
import signal
import atexit


# Explicitly enable required extensions
ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
    # "airlab.airstack",
    "omni.physx.forcefields",
    "omni.graph.core",                  # Core runtime for OmniGraph engine
    "omni.graph.action",                # Action Graph framework
    "omni.graph.action_nodes",          # Built-in Action Graph node library
    "omni.graph.ui",                    # UI scaffolding for graph tools
    "omni.graph.visualization.nodes",   # Visualization helper nodes
    "omni.graph.scriptnode",            # Python script node support
    "omni.graph.window.action",         # Action Graph editor window
    "omni.graph.window.generic",        # Generic graph UI tools
    "omni.graph.ui_nodes",              # UI node building helpers
    "airlab.pegasus",                   # Airlab extension Pegasus core extension
    "pegasus.simulator",
    "isaacsim.asset.exporter.urdf",     # URDF importer/exporter after prim is created
]:
    if not ext_manager.is_extension_enabled(ext):
        ext_manager.set_extension_enabled(ext, True)


class PegasusApp:
    """
    Minimal PegasusApp: just loads a Pegasus world and steps simulation
    """

    def __init__(self):
        # Timeline for controlling play/stop
        self.timeline = omni.timeline.get_timeline_interface()

        # Start Pegasus interface + world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load default environment. You can replace with url or file path of any desired environment.
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Flat Plane"])
        # self.pg.load_environment("/root/AirStack/assets/Scene_Templates_NVD@10011/Assets/Scenes/Templates/Basic/clean_cloudy_sky_and_floor.usd")
        
        # Spawn a PX4 multirotor drone with a specified vehicle ID and domain ID
        # PX4 udp port = 14540 + (vehicle_id)
        # Domain ID is for ROS2 domain communication. As of now, it should match the vehicle id by convention. 
        graph_handle = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor",
            drone_prim="/World/drone/base_link",
            vehicle_id=1,
            domain_id=1,
            usd_file="/root/Documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd",
            init_pos=[0.0, 0.0, 0.07],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

        # Down stereo ZED camera
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/drone/base_link",
            camera_name="DownZEDCamera",
            camera_offset = [0.0, 0.0, -0.05], # X, Y, Z offset from drone base_link
            camera_orientation_offset = [0.0, 90.0, 0.0], # Rotation (X, Y, Z) in degrees
            stereo_topic_namespace="down_stereo"
        )

        # Up stereo ZED camera
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/drone/base_link",
            camera_name="UpZEDCamera",
            camera_offset = [0.0, 0.0, 0.07], # X, Y, Z offset from drone base_link
            camera_orientation_offset = [0.0, -90.0, 0.0], # Rotation (X, Y, Z) in degrees
            stereo_topic_namespace="up_stereo"
        )

        # # Add an Ouster lidar (with an associated subgraph) to the drone
        # add_ouster_lidar_subgraph(
        #     parent_graph_handle=graph_handle,
        #     drone_prim="/World/drone/base_link",
        #     lidar_name="OS1_REV6_128_10hz___512_resolution",
        #     lidar_offset = [0.0, 0.0, 0.1], # X, Y, Z offset from drone base_link
        #     lidar_rotation_offset = [0.0, 0.0, 0.0], # Rotation in degrees
        # )
        
        # Reset so physics/articulations are ready
        self.world.reset()

        self.stop_sim = False


    def run(self):
        # Start sim timeline
        self.timeline.play()

        # Main loop
        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)

        # Cleanup
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()
