## **ENPM809B: Building a Manufacturing Robot Software System: RWA-3**

This is README to run the ROS package created for RWA3. 

## **Package Name: group7_rwa3**
This package consists of a listener (ROS Subcriber) to read the order data once the competition starts, get the sensor/camera data from the Ariac environment on the screen, detect the parts described in the order and ultimately pick up one of those parts from the conveyer belt. 


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

      `~/catkin_ws/src/group7_rwa3`
         
3. Source the setup.bash to add environment variables to your path to allow ROS to function

      `source /opt/ros/melodic/setup.bash`

4. Build your catkin workspace

      `cd ~/catkin_ws`
       
      `catkin_make`

  You can alternatively use following command to build only group7_rwa3 package.

      `catkin_make --only-pkg-with-deps group7_rwa3`

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
   
   `roslaunch group7_rwa3 group7_rwa3.launch`
   
   
   In terminal 2 (For starting motionplanning for arms through Moveit):

   `cd ~/catkin_ws`
    
   `source install/setup.bash`
    
   `roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1`

   In terminal 3:

   `cd ~/catkin_ws`
     
   `source devel/setup.bash`
    
   `rosrun group7_rwa3 listener`








