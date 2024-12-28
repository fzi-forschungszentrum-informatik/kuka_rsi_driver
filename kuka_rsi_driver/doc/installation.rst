.. _installation:

Installation
============

This driver is currently not distributed as a binary package, so you have to build it from sources. It is only tested on the current `jazzy <https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html>`_ and `rolling <https://docs.ros.org/en/rolling/Releases/Release-Rolling-Ridley.html>`_ distributions. In a sourced colcon workspace, the following steps are required:

1. Make sure ``vcs`` is installed:

   .. code-block:: console

     sudo apt install python3-vcstool

2. Clone relevant packages:

   .. code-block:: console

     git clone https://github.com/fzi-forschungszentrum-informatik/kuka_rsi_driver.git src/kuka_rsi_driver
     vcs import src --skip-existing --input src/kuka_rsi_driver/kuka_rsi_driver.${ROS_DISTRO}.repos

3. Clone robot description:

   In addition to this driver, an accurate robot description for your specific robot model is required. You should check if your description is already provided e.g. by `kuka_experimental`, otherwise you need to create a custom description.

   .. code-block:: console

     git clone https://github.com/StoglRobotics-forks/kuka_experimental.git -b rolling_overview src/kuka_experimental

4. Install all required dependencies:

   .. code-block:: console

     rosdep update
     rosdep install --from-paths src --ignore-src -ry

5. Build the driver:

   .. code-block:: console

     colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
