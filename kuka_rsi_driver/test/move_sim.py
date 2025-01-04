# Copyright 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import sys
import time
import unittest
import itertools
import os
import pytest
import rclpy
import rclpy.node
import launch_testing

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_testing.actions import ReadyToTest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

sys.path.append(os.path.dirname(__file__))
from test_interface import ControllerManagerInterface


@pytest.mark.launch_test
@launch_testing.parametrize(
    "prefix,robot_model",
    itertools.product(
        ["", "prefix_"],
        [
            ("kuka_kr5_support", "kr5_arc_macro.xacro", "kuka_kr5_arc"),
            ("kuka_kr120_support", "kr120r2500pro_macro.xacro", "kuka_kr120r2500pro"),
            ("kuka_kr210_support", "kr210r3100_macro.xacro", "kuka_kr210r3100"),
        ],
    ),
)
def generate_test_description(prefix, robot_model):
    kuka_rsi_driver = FindPackageShare("kuka_rsi_driver")

    test_description = LaunchDescription()

    #
    # Launch robot driver
    #
    robot_driver_launch = PathJoinSubstitution(
        [kuka_rsi_driver, "launch", "test_bringup.launch.py"]
    )
    test_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_driver_launch),
            launch_arguments={
                "description_package": robot_model[0],
                "description_macro_file": robot_model[1],
                "macro_name": robot_model[2],
                "launch_rviz": "False",
                "prefix": prefix,
            }.items(),
        )
    )

    #
    # Start simulator
    #
    test_description.add_action(
        Node(
            package="kuka_rsi_driver",
            executable="simulator",
            arguments=["--host-ip", "127.0.0.1"],
        )
    )

    test_description.add_action(ReadyToTest())
    return test_description


class MoveSimTest(unittest.TestCase):
    MOTION_CONTROLLERS = ["joint_trajectory_controller", "forward_position_controller"]

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.node.Node("kuka_rsi_driver_test")
        cls.controller_manager_interface = ControllerManagerInterface(cls.node)

        # Wait until spawners are done
        cls._wait_for_spawners(cls)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_activate_joint_trajectory_controller(self):
        self.controller_manager_interface.switch_controller(
            deactivate=self.MOTION_CONTROLLERS, strict=False
        )
        self.controller_manager_interface.switch_controller(
            activate=["joint_trajectory_controller"], strict=True
        )

    def test_activate_forward_position_controller(self):
        self.controller_manager_interface.switch_controller(
            deactivate=self.MOTION_CONTROLLERS, strict=False
        )
        self.controller_manager_interface.switch_controller(
            activate=["forward_position_controller"], strict=True
        )

    def _wait_for_spawners(self):
        expected_controllers_active = [
            "joint_state_broadcaster",
            "pose_broadcaster",
            "status_broadcaster",
            "joint_trajectory_controller",
        ]
        expected_controllers_inactive = ["forward_position_controller"]

        # Wait until all controllers are loaded
        for _ in range(10):
            controllers = self.controller_manager_interface.list_controllers()

            if len(controllers) == (
                len(expected_controllers_active) + len(expected_controllers_inactive)
            ):
                break

            time.sleep(1)

        # Verify controller activations
        for _ in range(5):
            # Check controllers
            controllers_active = [c.name for c in controllers if c.state == "active"]
            controllers_inactive = [
                c.name for c in controllers if c.state == "inactive"
            ]

            if (set(controllers_active) == set(expected_controllers_active)) and (
                set(controllers_inactive) == set(expected_controllers_inactive)
            ):
                return

            # Spawners are not yet done, so wait and retry
            time.sleep(1)
            controllers = self.controller_manager_interface.list_controllers()

        # Previous calls did not show controller activation success => Fail
        self.fail()
