# kuka_rsi_driver

This repository provides a [ros2_control](https://control.ros.org) based driver for kuka industrial
robot arms using the RSI protocol.

---
**NOTE**

This driver was tested on a limited set of robots (see [Tested with](#tested-with) section), but has to be considered experimental. In particular, the chosen communication structure should be more robust against disturbances, but is not thoroughly vetted.

---

## Build Status

<table width="100%">
    <tr>
        <th>
            <a href="https://docs.ros.org/en/jazzy">Jazzy</a>
        </th>
        <th>
            <a href="https://docs.ros.org/en/rolling">Rolling</a>
        </th>
    </tr>
    <tr>
        <td>
            <a href="https://github.com/fzi-forschungszentrum-informatik/kuka_rsi_driver/actions/workflows/jazzy_main.yml">
              <img src="https://github.com/fzi-forschungszentrum-informatik/kuka_rsi_driver/actions/workflows/jazzy_main.yml/badge.svg?branch=main"alt="Jazzy Main"/>
            </a> <br />
            <a href="https://github.com/fzi-forschungszentrum-informatik/kuka_rsi_driver/actions/workflows/jazzy_testing.yml">
              <img src="https://github.com/fzi-forschungszentrum-informatik/kuka_rsi_driver/actions/workflows/jazzy_testing.yml/badge.svg?branch=main"alt="Jazzy Testing"/>
            </a>
        </td>
        <td>
            <a href="https://github.com/fzi-forschungszentrum-informatik/kuka_rsi_driver/actions/workflows/rolling_main.yml">
              <img src="https://github.com/fzi-forschungszentrum-informatik/kuka_rsi_driver/actions/workflows/rolling_main.yml/badge.svg?branch=main"alt="Rolling Main"/>
            </a> <br />
            <a href="https://github.com/fzi-forschungszentrum-informatik/kuka_rsi_driver/actions/workflows/rolling_testing.yml">
              <img src="https://github.com/fzi-forschungszentrum-informatik/kuka_rsi_driver/actions/workflows/rolling_testing.yml/badge.svg?branch=main"alt="Rolling Testing"/>
            </a>
        </td>
    </tr>
</table>

## Packages in the Repository

  - `kuka_rsi_driver`: The `ros2_control` hardware interface for communicating with the robot.
  - `kuka_rsi_controllers`: Controllers for interfacing with vendor-specific functions of the robot.
  - `kuka_rsi_interfaces`: Interface definitions used in `kuka_rsi_controllers`.

## Getting Started

Perform the following steps to get started with this driver:

1. **Clone and build this driver**
   In addition to this driver, you need to find a robot description for the specific robot model you are using. If you are lucky, somebody already created one and shared it e.g. through kuka_experimental. This description lists the steps for a **KR 5 Arc** robot.

   ```bash
   sudo apt install python3-vcstool
   git clone https://github.com/fzi-forschungszentrum-informatik/kuka_rsi_driver.git src/kuka_rsi_driver
   git clone https://github.com/StoglRobotics-forks/kuka_experimental.git -b rolling_overview src/kuka_experimental
   vcs import src --skip-existing --input src/kuka_rsi_driver/kuka_rsi_driver.${ROS_DISTRO}.repos
   rosdep update
   rosdep install --from-paths src --ignore-src -ry
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   ```

2. **Setup the robot**
   Some configuration of the robot is required. The required steps are described in [setup documentation](kuka_rsi_driver/doc/setup.rst), please follow them carefully and verify you can reach the configured robot IP address from your control PC.

   If you want to test the driver without real hardware, a small simulator is provided that can be used instead.

3. **Start the driver**
   ```bash
   ros2 launch kuka_rsi_driver test_bringup.launch.py description_package:=kuka_kr5_support description_macro_file:=kr5_arc_macro.xacro macro_name:=kuka_kr5_arc rsi_listen_ip:=<Your host IP address>
   ```

   If you test the robot in simulation, set `rsi_listen_ip` to `127.0.0.1` and start the simulator:
   ```bash
   ros2 run kuka_rsi_driver simulator --host-ip 127.0.0.1
   ```

Take a look at [the documentation](kuka_rsi_driver/doc/index.rst) for information on how to get started with this driver.

## Features

This driver provides the following features:
- Joint position control and feedback.
- Publishing of joint torques.
- Publishing of Cartesian TCP pose.
- Speed scaling using slider on teach pendant (see KRL `$OV_PRO` variable).
- Robust fully asynchronous real-time communication.

## Tested With

So far, the driver was tested with the following robot models:
* **Agilus KR 10 R900 sixx** (KR C4 Controller)
* **IONTEC KR50 R2100** (KR C5 Controller)

## Acknowledgements

Development of this driver was in parts supported by the project "GANResilRob - Generative Adversarial Networks and Semantics for Resilient, Flexible Production Robots", funded by the German Federal Ministry for Economic Affairs and Climate Action.
