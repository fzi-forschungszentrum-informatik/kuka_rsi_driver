.. _overview:

Overview
========

This driver consists of three packages:

1. ``kuka_rsi_driver``: The main ros2_control hardware interface used for communicating with the robot.
2. ``kuka_rsi_controllers``: Controllers that expose vendor-specific functionality of the robot.
3. ``kuka_rsi_interfaces``: Interfaces used by ``kuka_rsi_controllers`` for interacting with vendor-specific functionality.

In order to get started using the driver, take a look at :ref:`getting_started`.

Features
--------

The driver provides some features in addition to joint-based position control:

* **Torque Feedback**: Measured torques from all joints are provided as ``<prefix>joint_a[1,...,6]/effort`` interfaces and published through the ``effort`` field of the ``joint_states`` message.
* **TCP Pose**: The TCP pose as provided by the robot is published through the ``tcp/position.x``, ..., ``tcp/orientation.w`` state interfaces. This considers any TCP offsets defined on the robot. This should be more accurate than a normal TF lookup, as the robot controller can consider the robot-specific kinematic calibration. The pose is exposed through a ``pose_broadcaster/PoseBroadcaster`` controller.
* **Speed Scaling**: The speed override as set by the KRL ``$OV_PRO`` variable (corresponds to the "Program Override" slider on the teach pendant) is provided through the ``speed_scaling/speed_scaling_factor`` state interface. It is set to zero if the program is currently not running. The ``kuka_rsi_controllers/StatusBroadcaster`` controllers publishes this to a ROS 2 topic.

  **NOTE**: This can be used by ``joint_trajectory_controller`` for runtime speed scaling, but this currently depends on the `Scaled JTC PR <https://github.com/ros-controls/ros2_controllers/pull/1191>`_ in ``ros2_controllers``. If you want to take advantage of this feature, you need to compile this PR by hand.

* **Robot State Publishing**: The current robot program state as defined in the KRL ``$PRO_STATE`` variable is provided in the ``robot_state/program_state`` state interface. It is published by the ``kuka_rsi_driver/StatusBroadcaster`` controller to a ROS 2 topic.

* **Simulator**: This driver provides a simple RSI simulator that allows testing robot setups.

* **Asynchronous Communication**: The realtime communication runs in an additional thread without any additional synchronization to the control thread. This should provide additional robustness against jitter in controller execution times or network delays.
