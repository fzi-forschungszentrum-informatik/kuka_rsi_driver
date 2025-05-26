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
import types

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)


def load_robot_description(path, args):
    content = Command(
        sum(
            [[FindExecutable(name="xacro"), " ", path]]
            + [[" ", f"{name}:=", value] for name, value in vars(args).items()],
            [],
        )
    )

    return {"robot_description": content}


def generate_launch_description():
    launch_description = LaunchDescription()
    kuka_rsi_driver = FindPackageShare("kuka_rsi_driver")

    def declare_launch_arg(name, **kwargs):
        launch_description.add_action(DeclareLaunchArgument(name, **kwargs))
        return LaunchConfiguration(name)

    robot_description_args = types.SimpleNamespace()

    def declare_robot_description_arg(name, **kwargs):
        value = declare_launch_arg(name, **kwargs)
        setattr(robot_description_args, name, value)
        return value

    launch_rviz = declare_launch_arg(
        "launch_rviz", default_value="true", description="Whether to start rviz"
    )
    rviz_config_file = declare_launch_arg(
        "rviz_config_file",
        default_value=PathJoinSubstitution(
            [kuka_rsi_driver, "etc", "visualization.rviz"]
        ),
        description="Absolute rviz config path to use",
    )

    declare_robot_description_arg(
        "description_package",
        description="Package to load robot description macro from",
    )
    declare_robot_description_arg(
        "description_macro_file",
        description="File name in $(find description_package)/urdf where robot macro "
        "is stored",
    )
    declare_robot_description_arg(
        "macro_name", description="Name of robot description macro"
    )

    declare_robot_description_arg(
        "use_mock_hardware",
        default_value="false",
        description="Whether to use mock hardware instead of real hardware interface",
    )
    declare_robot_description_arg(
        "mock_gpio_commands",
        default_value="false",
        description="If use_mock_hardware is set to true: Create additional command interfaces for faking GPIO states",
    )
    declare_robot_description_arg(
        "mock_sensor_commands",
        default_value="false",
        description="If use_mock_hardware is set to true: Create additional command interfaces for faking sensor measurements",
    )

    declare_robot_description_arg(
        "initial_positions_file",
        default_value=PathJoinSubstitution(
            [kuka_rsi_driver, "config", "initial_positions.yaml"]
        ),
        description="Initial joint positions when using mock hardware",
    )

    declare_robot_description_arg(
        "rsi_listen_ip",
        default_value="127.0.0.1",
        description="Own IP address to use for RSI communication",
    )
    declare_robot_description_arg(
        "rsi_listen_port",
        default_value="49152",
        description="Port to use for RSI communication",
    )

    declare_robot_description_arg(
        "prefix",
        default_value="",
        description="Prefix to add to each link and joint name",
    )

    #
    # Load robot description
    #
    description_path = PathJoinSubstitution(
        [kuka_rsi_driver, "urdf", "test_description.urdf.xacro"]
    )
    robot_description = load_robot_description(description_path, robot_description_args)

    #
    # Start driver
    #
    controllers_config_path = PathJoinSubstitution(
        [kuka_rsi_driver, "config", "controllers.yaml"]
    )
    launch_description.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[ParameterFile(controllers_config_path, allow_substs=True)],
        )
    )
    launch_description.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        )
    )

    #
    # Spawn controllers
    #
    def controller_spawner(names, activate=True):
        inactive_flag = ["--inactive"] if not activate else []
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=inactive_flag + names,
        )

    launch_description.add_action(
        controller_spawner(
            [
                "joint_state_broadcaster",
                "pose_broadcaster",
                "status_broadcaster",
                "joint_trajectory_controller",
            ]
        )
    )
    launch_description.add_action(
        controller_spawner(["forward_position_controller"], activate=False)
    )

    #
    # Start rviz
    #
    launch_description.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(launch_rviz),
            arguments=["-d", rviz_config_file],
        )
    )

    return launch_description
