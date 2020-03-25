# RWA 4: Build a complete Kit

### **How to build package**

Direct to your catkin_ws directory and run

 `catkin_make --only-pkg-with-deps group7_rwa4`

### **How to run**
To launch the environment, open a terminal and run

 `roslaunch group7_rwa4 group7_rwa4.launch`

Open two different terminals and run

 `roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1`
 
 `roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm2`

Open the 4th terminal and run

 `rosrun group7_rwa4 main_node`

Open 5th terminal, wait for all robots be stationary, then run

 `rosservice call /ariac/start_competition `

(readme author: An Lii)