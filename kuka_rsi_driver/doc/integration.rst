.. _integration:

Integrating the driver
======================

This driver provides an implementation of the RSI protocol, but no robot description or other robot-specific configuration for a specific robot arm. This means that it needs to be integrated into an existing robot model using the following steps.

Create initial robot description
--------------------------------

As we cannot distribute any original kuka robot meshes, an initial robot description is required. We recommend taking a look at `kuka_experimental <https://github.com/ros-industrial/kuka_experimental>`_ to check if your robot is already modeled there. Otherwise, you need to create a custom URDF from the manufacturer-supplied CAD data.

ros2_control URDF entries
-------------------------

This package provides an xacro macro for easier integration into an existing robot description. They can be added like the following:

.. code-block:: xml

  <xacro:include filename="$(find kuka_rsi_driver)/urdf/kuka_rsi_driver.ros2_control.xacro" />
  <xacro:kuka_rsi_driver_control
    name="<Name of the ros2_control system>"
    prefix="<joint name prefix>"
    rsi_listen_ip="<IP address of PC running the driver>"
    rsi_listen_port="<Optional port, defaults to 49152>"
  />

This expects the joints in the urdf to be named ``<prefix>joint_a[1,...,6]``.

Launch file integration
-----------------------

This driver provides a controller config file that can be used as a starting point. In order to use them, you need the following components in your launch file:

.. code-block:: python

  # Robot state publisher
  launch_description.add_action(
      Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
      )
  )

  # Start controller_manager node
  controllers_config_path = PathJoinSubstitution(
      [FindPackageShare("kuka_rsi_driver"), "config", "controllers.yaml"]
  )
  launch_description.add_action(
      Node(
          package="controller_manager",
          executable="ros2_control_node",
          output="screen",
          parameters=[ParameterFile(controllers_config_path, allow_substs=True)],
      )
  )

  # Start active controllers
  launch_description.add_action(
      Node(
          package="controller_manager",
          executable="spawner",
          output="screen",
          arguments=[
              "joint_state_broadcaster",
              "pose_broadcaster",
              "status_broadcaster",
              "joint_trajectory_controller",
          ],
      )
  )

  # Start inactive controllers
  launch_description.add_action(
      Node(
          package="controller_manager",
          executable="spawner",
          output="screen",
          arguments=[
            "forward_position_controller",
          ],
      )
  )
