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
from test_interface import ControllerManagerInterface, ActionInterface


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

    def test_move_jtc(self, prefix):
        # Make sure jtc is active
        self.controller_manager_interface.switch_controller(
            deactivate=self.MOTION_CONTROLLERS, strict=False
        )
        self.controller_manager_interface.switch_controller(
            activate=["joint_trajectory_controller"], strict=True
        )

        # Create dummy trajectory
        joint_names = [f"{prefix}joint_a{i+1}" for i in range(6)]
        test_points = [
            (Duration(sec=2, nanosec=0), [0.0 for j in joint_names]),
            (Duration(sec=4, nanosec=0), [-1.0 for j in joint_names]),
            (Duration(sec=8, nanosec=0), [1.0 for j in joint_names]),
        ]
        test_trajectory = JointTrajectory(
            joint_names=joint_names,
            points=[
                JointTrajectoryPoint(positions=pos, time_from_start=time)
                for (time, pos) in test_points
            ],
        )

        # Send goal to jtc
        follow_joint_trajectory_client = ActionInterface(
            "joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
            self.node,
            3,
        )
        result = follow_joint_trajectory_client.send_goal(
            FollowJointTrajectory.Goal(trajectory=test_trajectory), timeout=9
        )
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)

        # Verify joint state
        joint_state = self._subscribe_once("/joint_states", JointState, 3)

        expected_state = test_points[-1][1]
        for i, joint_name in enumerate(joint_names):
            joint_i = joint_state.name.index(joint_name)
            self.assertLess(
                abs(expected_state[i] - joint_state.position[joint_i]), 0.01
            )

    def _subscribe_once(self, topic, topic_type, timeout=10):
        # Create subscriber
        last_msg = None

        def msg_cb(msg):
            nonlocal last_msg
            last_msg = msg

        subscription = self.node.create_subscription(JointState, topic, msg_cb, 1)

        # Spin until receiving msg
        self.node.get_logger().info(
            f"Waiting for message on topic {topic} ({topic_type.__name__}) for {timeout:.2f}s"
        )
        spin_start = time.time()
        while (last_msg is None) and (time.time() < (spin_start + timeout)):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertIsNotNone(last_msg)
        self.node.get_logger().info(f"  Received message: {last_msg}")

        # Cleanup
        subscription.destroy()
        return last_msg

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
