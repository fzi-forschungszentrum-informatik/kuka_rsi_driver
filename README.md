# kuka_rsi_driver

This repository provides a [ros2_control](https://control.ros.org) based driver for kuka industrial
robot arms using the RSI protocol.

---
**NOTE**

This driver was tested on a limited set of robots (see [Tested with](#tested-with) section), but has to be considered experimental. In particular, the chosen communication structure should be more robust against disturbances, but is not thoroughly vetted.

---

## Packages in the Repository

  - `kuka_rsi_driver`: The `ros2_control` hardware interface for communicating with the robot.
  - `kuka_rsi_controllers`: Controllers for interfacing with vendor-specific functions of the robot.
  - `kuka_rsi_interfaces`: Interface definitions used in `kuka_rsi_controllers`.

## Getting Started

This driver only provides an implementation of the robot realtime communication, but no specific robot support package. Take a look at [the documentation](kuka_rsi_driver/doc/index.rst) for informations on how to get started and integrate it into an existing setup.

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
