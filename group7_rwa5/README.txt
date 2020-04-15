## **ENPM809B: Building a Manufacturing Robot Software System: RWA-5**

This is README to run the ROS package created for RWA5.

## **Package Name: group7_rwa5**
* This package builts an entire kit for the RWA5.
* It builds a whole kit based on the order.
* It has the abilty to detect faulty parts and replace them.
* It also checks for the updates to an order.
* It takes smart decisions between modifying the current kit or building a whole new kit if that order is updated.
* The robotic arms help each other if one of them is not able to reach a part while building a kit.
## **Minimum System Requirements**
  * ROS Melodic
  * Gazebo >= 9.6.0
  * Ariac 2019
  * Ubuntu Desktop 18.04 Bionic (64-bit)
  * moveit
  * gazebo_ros_pkgs

## gazebo_ros_pkgs
gazebo_ros_pkgs package should be available in the workspace to run group7_rwa5. Use the following command to
clone the package and get it working

`cd ~/catkin_ws/src`

`git clone https://github.com/osrf/ariac-gazebo_ros_pkgs -b ariac-network-melodic`

`cd ~/catkin_ws`

`catkin_make --only-pkg-with-deps gazebo_ros_pkgs`

Read more at https://bitbucket.org/osrf/ariac/wiki/2019/tutorials/installation

## moveit_visual_tools
You should also have moveit_visual_tools installed if not installed previously. moveit_visual_tools is not installed by default using installation instructions on ariac 2019 website. Use the below command to install moveit_visual_tools for ros-melodic

  `sudo apt-get install ros-melodic-moveit-visual-tools`

## **Instructions to RUN the package**
1. Create and build a catkin workspace

      `mkdir -p ~/catkin_ws/src`

      `cd ~/catkin_ws/`

2.  Extract the package to `~/catkin_ws/src/` as below

      `~/catkin_ws/src/group7_rwa5`

3. Source the setup.bash to add environment variables to your path to allow ROS to function

      `source /opt/ros/melodic/setup.bash`
      `source ~/catkin_ws/devel/setup.bash`

4. Build your catkin workspace

      `cd ~/catkin_ws`

      `catkin_make`

    You can alternatively use following command to build only group7_rwa5 package.

    `catkin_make --only-pkg-with-deps group7_rwa5`

  Note: Always call `catkin_make` in the root of your catkin workspace.

5. Overlay your catkin workspace on top of your environment.

     `cd ~/catkin_ws`

     `source devel/setup.bash`

## **Launch the Package**

To run the environment and the node, open another terminal and run the following command
Ensure that you are inside your workspace directory

In terminal 1:

	`cd ~/catkin_ws`

	`source devel/setup.bash`

	`roslaunch group7_rwa5 group7_rwa5.launch`


In terminal 2 (to use moveit interface for arm1):

	`cd ~/catkin_ws`

	`source install/setup.bash`

	`roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1`

In terminal 3 (to use moveit interface for arm2):

	`cd ~/catkin_ws`

	`source install/setup.bash`

	`roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm2`

In terminal 4:

	`cd ~/catkin_ws`

	`source devel/setup.bash`

	`rosrun group7_rwa5 main_node`