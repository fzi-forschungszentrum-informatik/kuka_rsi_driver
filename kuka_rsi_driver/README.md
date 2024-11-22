kuka_rsi_driver
===============

This package provides a [ros2_control](https://control.ros.org) hardware interface for kuka industrial robot arms using the RSI protocol.

---
**NOTE**

This driver only provides a control implementation, but no robot description or other robot specific configuration. This means that you already need a correct model of your robot to be able to use this.

---

Interfaces
----------

In addition to position-based control and joint torque feedback, the driver provides the following interfaces:
- `tcp/position.x`, ..., `tcp/orientation.w`: The robot end effector pose (including any tool offsets configured on the robot) relative to `base_link`. As this is directly taken from the robot, it should be more accurate than querying TFs published by `robot_state_publisher`.
- `speed_scaling/speed_scaling_factor`: Speed override set by the program override slider on the teach pendant (see `$OV_PRO` KRL variable), scaled between zero and one. When the program is stopped, this is set to zero. This can be used together with `joint_trajectory_controller` to enable easy runtime speed adjustments and correct continuations after program stops.
- `robot_state/program_state`: The program interpreter state as defined in `$PRO_STATE`. The values should be consistent with the constants defined in the [`ProgramState`](kuka_rsi_interfaces/msg/ProgramState.msg) message.

Simulator
---------

This package contains a simple RSI simulator. In order to use it, start your robot driver with `rsi_listen_ip` set to `0.0.0.0` and then run
```console
ros2 run kuka_rsi_driver simulator --host-ip 127.0.0.1
```
