kuka_rsi_controllers
====================

This packages contains controllers for interacting with robot-specific aspects of the driver. Currently this only contains:
- `kuka_rsi_controllers/StatusBroadcaster`: Broadcasts the current robot state and speed scaling to topics.

kuka_rsi_controllers/StatusBroadcaster
--------------------------------------

This controller publishes the robot-specific robot state in every control cycle.

Used interfaces:
- `robot_state/program_state`
- `speed_scaling/speed_scaling_factor`

Provided interfaces:
- Topic `~/robot_state`: `kuka_rsi_interfaces::msg::RobotState`. Tries to publish current robot state in every controller cycle.
- Topic `~/speed_scaling`: `std_msgs::msg::Float64`. Tries to publish current program speed override in every controller cycle.
