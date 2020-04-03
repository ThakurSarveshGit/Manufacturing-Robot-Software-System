## **ENPM809B: Building a Manufacturing Robot Software System: RWA-4**

This is README to run the ROS package created for RWA4. 

## **Package Name: group7_rwa4**
This package builts an entire kit for the RWA4. It detects for faulty parts and replaces them.


## **Minimum System Requirements**
  * ROS Melodic 
  * Gazebo >= 9.6.0 
  * Ariac 2019
  * Ubuntu Desktop 18.04 Bionic (64-bit)
  * moveit

## moveit_visual_tools
You should also have moveit_visual_tools installed if not installed previously. moveit_visual_tools is not installed by default using installation instructions on ariac 2019 website. Use the below command to install moveit_visual_tools for ros-melodic

  `sudo apt-get install ros-melodic-moveit-visual-tools`

## **Instructions to RUN the package**
1. Create and build a catkin workspace

      `mkdir -p ~/catkin_ws/src`
       
      `cd ~/catkin_ws/`

2.  Extract the package to `~/catkin_ws/src/` as below

      `~/catkin_ws/src/group7_rwa4`
         
3. Source the setup.bash to add environment variables to your path to allow ROS to function

      `source /opt/ros/melodic/setup.bash`
      `source ~/catkin_ws/devel/setup.bash`

4. Build your catkin workspace

      `cd ~/catkin_ws`

      `catkin_make --only-pkg-with-deps group7_rwa4`

  You can alternatively use following command to build only group7_rwa3 package.

      `catkin_make --only-pkg-with-deps group7_rwa4`

  Note: Always call `catkin_make` in the root of your catkin workspace. 

5. Overlay your cactkin workspace on top of your environment.

     `cd ~/catkin_ws`

     `source devel/setup.bash`
 
## **Launch the Package**

To run the environment and the listener node, open another terminal and run the following command
Ensure that you are inside your workspace directory
   In terminal 1:

	`cd ~/catkin_ws`

	`source devel/setup.bash`

	`roslaunch group7_rwa4 group7_rwa4.launch`


	In terminal 2 (For starting motionplanning for arms through Moveit):

	`cd ~/catkin_ws`

	`source install/setup.bash`

	`roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1`

	In terminal 3 (For starting motionplanning for arms through Moveit):

	`cd ~/catkin_ws`

	`source install/setup.bash`

	`roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm2`

	In terminal 4:

	`cd ~/catkin_ws`
	 
	`source devel/setup.bash`

	`rosrun group7_rwa4 main_node`








