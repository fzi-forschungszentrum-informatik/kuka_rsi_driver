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
from math import radians

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_testing.actions import ReadyToTest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

sys.path.append(os.path.dirname(__file__))
from test_interface import ControllerManagerInterface, ActionInterface, subscribe_once


@pytest.mark.launch_test
@launch_testing.parametrize(
    "prefix,robot_model,use_mock_hardware",
    itertools.product(
        ["", "prefix_"],
        [
            ("kuka_kr5_support", "kr5_arc_macro.xacro", "kuka_kr5_arc"),
            ("kuka_kr120_support", "kr120r2500pro_macro.xacro", "kuka_kr120r2500pro"),
            ("kuka_kr210_support", "kr210r3100_macro.xacro", "kuka_kr210r3100"),
        ],
        [True, False],
    ),
)
def generate_test_description(prefix, robot_model, use_mock_hardware):
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
                "use_mock_hardware": f"{use_mock_hardware}",
                "launch_rviz": "False",
                "prefix": prefix,
            }.items(),
        )
    )

    #
    # Start simulator
    #
    if not use_mock_hardware:
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

        MoveSimTest.test_cnt = 1
        cls.log = cls.node.get_logger()

        # Wait until spawners are done
        cls._wait_for_spawners(cls)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.log = self.node.get_logger().get_child(f"{MoveSimTest.test_cnt}")
        MoveSimTest.test_cnt += 1

        print("\n")
        self.log.info(f"Running test {self.id()}")

    def test_activate_motion_controllers(self):
        for controller in self.MOTION_CONTROLLERS:
            with self.subTest(controller=controller):
                self.log.info(f"Testing activation of motion controller '{controller}'")
                self._activate_motion_controller(controller)

                time.sleep(1)

                self.log.info(f"Verifying state of '{controller}'")
                controllers = self.controller_manager_interface.list_controllers()
                active_controllers = [
                    c.name for c in controllers if c.state == "active"
                ]
                self.assertIn(controller, active_controllers)
                self.log.info(f"Successfully activated '{controller}'")

    def test_move_jtc(self, prefix):
        self._activate_motion_controller("joint_trajectory_controller")

        # Create dummy trajectory
        joint_names = [f"{prefix}joint_a{i+1}" for i in range(6)]
        test_points = [
            (Duration(sec=2, nanosec=0), [0.0 for j in joint_names]),
            (
                Duration(sec=4, nanosec=0),
                [radians(v) for v in [-90, -120, -10, -270, -90, -270]],
            ),
            (
                Duration(sec=6, nanosec=0),
                [radians(v) for v in [90, -20, 120, 270, 90, 270]],
            ),
        ]
        test_trajectory = JointTrajectory(
            joint_names=joint_names,
            points=[
                JointTrajectoryPoint(positions=pos, time_from_start=time)
                for (time, pos) in test_points
            ],
        )

        # Send goal to jtc
        self.log.info("Sending dummy trajectory to controller")
        follow_joint_trajectory_client = ActionInterface(
            "joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
            self.node,
            3,
        )
        result = follow_joint_trajectory_client.send_goal(
            FollowJointTrajectory.Goal(trajectory=test_trajectory), timeout=7
        )
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)

        # Verify joint state
        time.sleep(1)
        self.log.info("Verifying final position using joint_states")
        joint_state = subscribe_once(
            "/joint_states", JointState, self.node, 3, self.log
        )

        expected_state = test_points[-1][1]
        for i, joint_name in enumerate(joint_names):
            joint_i = joint_state.name.index(joint_name)
            self.assertLess(
                abs(expected_state[i] - joint_state.position[joint_i]), 0.01
            )
        self.log.info("Motion successfull")

    def _wait_for_spawners(self):
        expected_controllers_active = [
            "joint_state_broadcaster",
            "pose_broadcaster",
            "status_broadcaster",
            "joint_trajectory_controller",
        ]
        expected_controllers_inactive = ["forward_position_controller"]

        # Wait until all controllers are loaded
        self.log.info("Waiting for all controllers to be loaded")
        for _ in range(10):
            controllers = self.controller_manager_interface.list_controllers()

            if len(controllers) == (
                len(expected_controllers_active) + len(expected_controllers_inactive)
            ):
                break

            self.log.info("Not all controllers are loaded yet - retrying")
            time.sleep(1)

        # Verify controller activations
        self.log.info("Waiting for all spawners to be done")
        for _ in range(5):
            # Check controllers
            controllers_active = [c.name for c in controllers if c.state == "active"]
            controllers_inactive = [
                c.name for c in controllers if c.state == "inactive"
            ]

            if (set(controllers_active) == set(expected_controllers_active)) and (
                set(controllers_inactive) == set(expected_controllers_inactive)
            ):
                self.log.info("All controllers are in their expected state")
                return

            # Spawners are not yet done, so wait and retry
            self.log.info("Not all controllers are fully spawned yet - retrying")
            time.sleep(1)
            controllers = self.controller_manager_interface.list_controllers()

        # Previous calls did not show controller activation success => Fail
        self.fail()

    def _activate_motion_controller(self, controller):
        self.log.info(
            f"Activating motion controller '{controller}', deactivating {self.MOTION_CONTROLLERS} first"
        )
        self.controller_manager_interface.switch_controller(
            deactivate=self.MOTION_CONTROLLERS, strict=False
        )
        self.controller_manager_interface.switch_controller(
            activate=[controller], strict=True
        )
        self.log.info("  done")
