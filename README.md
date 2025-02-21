# Welcome to My Autonomous ROS2 Robot Simulation!
I have another repository called "neobot" where I am trying to implement many different things. I am using the same robot but this repository is much cleaner and created for the assignment specifically.


This repository is a ros2 workspace consisting packages for a differential drive robot simulation using **ROS2 Jazzy** and **Gazebo Harmonic** running on Ubuntu 24.04.
## [Demonstration Video](https://drive.google.com/file/d/1oKI765Btkg7Z4Wbc1PxxNoXfpkGyQTXi/view?usp=drive_link)
https://drive.google.com/file/d/1oKI765Btkg7Z4Wbc1PxxNoXfpkGyQTXi/view?usp=drive_link


## Dependencies:
ROS2 Jazzy, Gazebo Harmonic

    sudo apt-get update
Install the interfaces:

    sudo apt-get install ros-jazzy-ros-gz

Install remaining dependencies using rosdep command in the workspace root directory:

    rosdep install --from-paths src --ignore-src -r -y

**NOTE**: The workspace also contains "gz_ros2_control" package which I had to build from source due to some bugs

## Build the workspace
In the root directory of the workspace

    colcon build
Source the workspace.

    source ~/akarsh_nav2_ws/install/setup.bash
## Run
### For C++ node to publish initial pose, send goal and publish markers
Launch the sim:

    ros2 launch neobot neobot_gz.launch.py
Start Nav2:

    ros2 launch neobot_nav2 neobot_nav2.launch.py
Run C++ node to initialize pose, send goal, publish markers:

    ros2 run neobot_nav2 nav2_control
Initial pose (0,0,0) gets sent automatically as the node starts, then send goal according to the prompt.

### For C++ node for multi-goal navigation
Launch the sim:

    ros2 launch neobot neobot_gz.launch.py
Start Nav2:

    ros2 launch neobot_nav2 neobot_nav2.launch.py
Run C++ node to initialize pose, send waypoints:

    ros2 run neobot_nav2 multi_goal
Initial pose (0,0,0) gets sent automatically as the node starts, then enter the number of waypoints, and waypoints according to the prompt.



## Nav2 Parameters tuning observations
### AMCL
### Global Planner
### Local Planner
### Costmaps

## Sensor Fusion of LiDAR and Camera PointCloud
I tried to use the PointCloud data with my 2D Lidar data to update the costmaps, however the PointCloud data was not being used, I checked the PointCloud topic, the rate of the topic, the frame_id, etc and all seem to be correct, here is the parameter snippet.

    observation_sources: scan rgbd_cloud

    scan:    
	    topic: /scan    
	    max_obstacle_height: 2.0    
	    clearing: True    
	    marking: True    
	    data_type: "LaserScan"
	    raytrace_max_range: 3.0    
	    raytrace_min_range: 0.0    
	    obstacle_max_range: 2.5    
	    obstacle_min_range: 0.0    
    
    rgbd_cloud:    
	    topic: /depth_camera/points
	    max_obstacle_height: 3.0    
	    min_obstacle_height: 0.1
	    raytrace_max_range: 5.0
	    raytrace_min_range: 0.0
	    obstacle_max_range: 5.5
	    obstacle_min_range: 0.0
	    clearing: True
	    marking: True
	    data_type: "PointCloud2"

## Approach to localization, path planning and obstacle avoidance
I used the default AMCL algorithm for localization, using a prebuilt map created using the slam_toolbox package, the AMCL algorithm uses the LiDAR data and scan matching to provide a transform from the map to odom frames since the odometry drifts over time. We provide an initial pose estimate to amcl.

For the path planning, I used the default NavFn planner which makes use of Djikstra's shortest path algorithm. For local planner, I used the Regulated Pure Pursuit (RPP) controller which I was able to tune to make the robot follow the path as closely as possible. We can also tune it in such a way where the robot cuts corners, and uses lesser "rotate to heading" which is faster and smoother but not suitable for tight environments.

For obstacle avoidance, I use observation sources like Lidar to update the global costmap and the local costmap. Updating the global costmap through the obstacle_layer plugin allows to re-plan the path around the obstacle, and updating the local costmap using voxel_layer plugin allows the controller to make adjustments.
Also RPPs lookahead parameters help in slowing down the robot ahead of time.
I changed parameters like inflation_radius, cost factors etc to control how close the robot can go to obstacles.
I tried to use depth camera's PointCloud data also but was unable to get that working.

## Challenges faced and solutions
The first challenge I faced was poor localization, I solved this to some extent by tuning the alpha parameters of the amcl node. The 5 alpha parameters are basically how confident the the node should be in different scenarios, I have explained this in the parameter tuning section.

Second challenge was creating the c++ nodes, using the service callback and action callback functions since they are non blocking functions, the script would prompt to send the goal position even though the amcl node wasn't ready and the initial pose wasn't set. It would also prompt for the next goal even if the current goal isn't complete yet. To solve this, I had to use callback functions and call the set_initial_pose, publish_goal functions inside the callback functions.

Another challenge I faced was that the costmaps are not updating using the PointCloud data from the depth camera. I haven't been able to find a solution for this.

One more challenge faced was that the robot was not following the global path accurately, cutting corners around tight turns with obstacles around would make the robot stall and enter recovery mode. This was solved by tuning the lookahead parameters of the RPP controller and rotate to heading parameter to make the robot follow the path more closely however at the cost of speed and smoothness.
