# Overview
This collection of packages aims to provide robust, laser-based, indoor navigation capabilities for a turtlebot mobile robot platform. Four packages are provided:
- robust_navigation: Provides launch files and configuration files for navigation packages
- robust_controller: Provides actionlib-based controller with the same interface and move_base, but with more robust recovery options
- map_labelling: Provides services for semantic labeling of locations
- hector_navigation: A subset of the [hector_navigation](http://wiki.ros.org/hector_navigation) packages with some custom adjustments for autonomous exploration of an unkown environment.

# Dependencies
- ROS [Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) - Full Desktop installation recommended
- For core navigation capabilities
    - Google's [Cartographer for Turtlebot](https://google-cartographer-ros-for-turtlebots.readthedocs.io/en/latest/)
        - See the [Cartographer Installation Notes](#cartographer-installation-notes) section for installation and troubleshooting tips
    - Turtlebot packages: `sudo apt-get install ros-indigo-turtlebot`
    - Turtlebot navigation packages: `sudo apt-get install ros-indigo-turtlebot-navigation`
    - TEB local planner: `sudo apt-get install ros-indigo-teb-local-planner`
    - amcl: `sudo apt-get install ros-indigo-amcl`
    - move_base: `sudo apt-get install ros-indigo-move-base`
- For running a Hokuyo laser, the hokuyo_node package is needed: `sudo apt-get install ros-indigo-hokuyo-node`
- For autonomus exploration hector_slam is needed: `sudo apt-get install ros-indigo-hector-slam`
- For the robust controller, SMACH is needed: `sudo apt-get install ros-indigo-smach-ros`
- Even if you aren't planning to use all of the capabilites of this packages, it is recommended that you install all of the above dependencies, especially if you are new to ROS. If you are an advanced ROS user, you are welcome to modify the packages and launch files to bypass dependencies that you will not need. See the Advanced Notes section below for more details.


# Installation
1. Ensure all required dependencies are installed
2. Clone this repository to your computer and move the packages to your catkin workspace. For information on setting up a catkin workspace, refer to the instructions [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
3. Run catkin_make at the top level of your workspace to compile all of the code and messages
4. Source the setup script with `source devel/setup.bash`. You can add `source ~/path/to/workspace/devel/setup.bash` to your .bashrc file to auto-source this workspace in the future.
5. Copy updated turtlebot_library URDF from robust_navigation/UpdatedTurtlebotFiles and put it in $(find turtlebot_description)/urdf. You can use `roscd turtlebot_description/urdf` to get to the right folder.  This file adds in the description of the lidar placement so the transforms are handled properly. If your lidar is not placed as shown in the image below, you will need to edit the urdf to ensure it matches your lidar placement.

<img src="https://cloud.githubusercontent.com/assets/11199681/25629217/cac891fa-2f36-11e7-878a-06041fe02909.png"  width="250">


# Basic Usage

## Building Maps
Google Cartographer is used for SLAM and is configured to take advantage of the excellent turtlebot odometry. The map building can be done autonomously by exploring unknown space or via teleoperation. Mapping can also be done offline if data is collected via [rosbag](http://wiki.ros.org/rosbag).

A few sample maps are provided in the robust_navigation/maps directory but you are encouraged to build your own for the space and use case you are working with.

- Teleoperated map building
    - Start teleoperation nodes in separate terminals
        - `roslaunch robust_navigation basic_mapping.launch`
        - `roslaunch turtlebot_teleop keyboard_teleop.launch`
        - Follow instructions in terminal to drive
    - Visualize or save map if desired (see below)
       
- Offline map building
    - Launch cartographer node with `roslaunch robust_navigation cartographer.launch`
    - Play collected rosbag data
    - You may need to change the value of use_sim_time in cartographer.launch, depending on how your system is handling the clock
    - Visualize or save map if desired (see below)
    
- Autonomous map building
    - Note: This uses the hector_exploration_planner which has several known issues. One is that on startup, the robot will not move. It needs to be given a small turn or push for the first section of the map to be built so the planner can start working. Additionally, the planner is fairly slow and inefficient as it was built for search and rescue applications. Hopefully, a new and more effective exploration planner can be used in the future. See the [Hector Navigation Notes](#hector-navigation-notes) section below for more details.
    - Run with: `roslaunch robust_navigation autonomous_mapping.launch`       
    - Visualize or save map if desired (see below)

- To visualize map during SLAM
    - Run: `roslaunch robust_navigation view_navigation.launch`
    - If you are using ssh to connect to the robot you must set up your computer with ros, have a properly [configured network](http://wiki.ros.org/ROS/NetworkSetup), and run the visualization on your own computer. It will not work over ssh.
    
- To save a map after SLAM
    - Save map once built (but before killing the mapping node!) with `rosservice call finish_trajectory <map_name>`. Files will usually end up in ~/.ros. You can move them to a new folder with `mv ~/.ros/<map_name>.* /path/to/maps/folder`. See the [Cartographer API](https://google-cartographer-ros.readthedocs.io/en/latest/ros_api.html) for more info.

## Localization + Navigation
- Launch with: `roslaunch map_navigation.launch`
- Set the desired map in the launch file by changing the name of the map_file parameter
    - You can also publish map with: `rosrun map_server map_server <file path>`
- Current pose is published as geometry_msgs/PoseWithCovarianceStamped on /amcl_pose
- Must provide initial pose as geometry_msgs/PoseWithCovarianceStamped on /initialpose
    - The initial pose can also be hard coded in the launch file. See robust_navigation/launch/amcl.launch for an example of how to do this.
- Send a goal as a MoveBaseGoal
    - See example [here](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals)
- Can use rviz to visualize navigation as well as set initial pose and goals
    - `roslaunch turtlebot_rviz_launchers view_navigation.launch`
    - If you are using ssh to connect to the robot you must set up your computer with ros, have a properly [configured network](http://wiki.ros.org/ROS/NetworkSetup), and run the visualization on your own computer. It will not work over ssh.
    - On the top toolbar there are buttons for setting the robot's initial pose and sending a navigation goal. These are great for testing navigation and localization.
    
## Robust Controller
The robust_controller can be used as a substitute for move_base. It can be launched with `roslaunch robust_controller robust_controller.launch`. It integrates with actionlib and takes the same goal as move_base. The result value is returned as an integer according to:

- 0 = default/not started
- 1 = arrived at the goal successfully
- 2 = arrived at the goal, but had to execute one or more recovery states along the way
- 3 = aborted goal and returned to home position
- 4 = aborted goal, could not return to home position, stuck somewhere

An example code snippet is shown below:

```python
from robust_controller.msg import RobustControllerGoal, RobustControllerAction
import actionlib

#set up action client
client = actionlib.SimpleActionClient('robust_controller_server',RobustControllerAction)
client.wait_for_server()

#send a new goal
actionGoal = RobustControllerGoal()
actionGoal.header = my_header
actionGoal.target_pose = my_target_pose
client.send_goal(actionGoal) 
client.wait_for_result()
rslt = client.get_result()
print "Robust Controller Result is: "+str(rslt.result)
```

If you wish to modify the recovery behaviors of robust_controler, see the [Robust Controller Details](#robust-controller-details) sections below.


## Semantic Map Labelling
We have an additional service that wraps around the above services that provides the ability to interact with the map at a higher level. This service requires the localization node to be running.

To use: 
- Launch the services with `roslaunch map_labelling map_labelling.launch`

To interact, we have serveral service calls: 
- `rosservice call label_map "[LOCATION_LABEL]"` labels the current robot location with the given name 
- `rosservice call get_nearest_location` gives the label name of the nearest location, and the x and y distance to that location
- `rosservice call goto_location "[LOCATION_LABEL]"` will command the robot to go to the location with the given name
- `rosservice call distance_to_location "[LOCATION_LABEL]"` returns the distance from the the robot to the specified location with the given name
- `rosservice call motion_status` returns the status of the current motion as an integer according to the specifications [here](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html). Note this must be called AFTER goto_location has been called at least once.

The following lines show how to take advantage of this service in a script:
```python
rospy.wait_for_service('goto_location')  
try:    
    goto_service = rospy.ServiceProxy('goto_location', GoToLocation)  
    resp = goto_service("[LOCATION_LABEL]")  
except rospy.ServiceException, e:  
    print "Service call failed: %s" % e
```

# Advanced Notes
Below you will find various notes, tips, and details about important sections of this package

## Launch File Details
- robust_navigation
    - amcl.launch, cartographer.launch, lidar.launch, move_base.launch: launch exactly what they say
        - Note: if using a lidar that is not compatible with the hokuyo_node package, you need to modify the lidar.launch file to use the correct package
    - basic_mapping.launch: sets up the required nodes for mapping
    - autonomus_mapping.launch: starts mapping and nodes for autonomus exploration
    - map_navigation.launch: loads a previously saved map and runs nodes for navigation
    - torture_test.launch: runs a navigatoin torture test with a previously saved map and labelled locations
    - view_mapping.launch: open rviz configured for viewing cartographer mapping progress
    - view_navigation.launch: open rviz configured for viewing navigation progress
- robust_controller
    - robust_controller.launch: sets up the action server for the robust controller
- map_labelling
    - map_labelling.launch: starts the map labelling services. This also starts the robust_controller server since the goto_label service uses the robust controller. 
- hector_navigation/hector_exploration_node
    - exploration_planner.launch: starts the planner node which provides a ROS service for calling the exploration planner
    
## Parameter Tuning
Much of the work in this package was simply finding and understanding the parameters that needed to be changed to increase the robustness of the navigation system so here is a list of some of the key parameters and links to useful sources that helped a ton. 

Most of the parameter values can be found in the corresponding parameter file in robust_navigation/params. The one exception is amcl whose parameters are in the launch file.

- cartographer
    - [Official parameter documentation](https://google-cartographer.readthedocs.io/en/latest/configuration.html) - though at the time of writing many of the parameters were not fully documented or explained.
    - Odometry model weights - increase to trust odometry more
        - TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight
        - TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight
    - Confidence threshold for scan matching - increase to require scans to match better before trusting them
        - SPARSE_POSE_GRAPH.constraint_builder.min_score
    - Submap resolution - decrease to get more fine grained features and therfore better scan matches
        - TRAJECTORY_BUILDER_2D.submaps.resolution
    - Useful Github issues related to tuning
        - https://github.com/googlecartographer/cartographer_ros/issues/249
        - https://github.com/googlecartographer/cartographer/issues/141
        - https://github.com/googlecartographer/cartographer/issues/127
        - https://github.com/googlecartographer/cartographer_turtlebot/issues/48
- teb_local_planner
    - [Official parameter documentation](http://wiki.ros.org/teb_local_planner) - very well documented
    - [Official tutorials](http://wiki.ros.org/teb_local_planner/Tutorials)
    - [FAQ with useful tuning tips](http://wiki.ros.org/teb_local_planner/Tutorials/Frequently%20Asked%20Questions)
    - Basic configuration - critical to set these appropriately for your robot
        - max_vel_x
        - max_vel_x_backwards
        - max_vel_theta
        - acc_lim_x
        - acc_lim_theta
        - min_turning_radius
        - footprint_model
     - Obstacle avoidance - see the [FAQ](http://wiki.ros.org/teb_local_planner/Tutorials/Frequently%20Asked%20Questions) for more details about obstacle avoidence tuning and corner cutting
        - min_obstacle_dist
        - inflation_dist
        - global_plan_viapoint_sep
        - weight_inflation
        - weight_viapoint
     - CPU usage and multi-path planning - again the [FAQ](http://wiki.ros.org/teb_local_planner/Tutorials/Frequently%20Asked%20Questions) addresses some other parameters to help solve this issue however, even after trying every suggestion on the list we found the only thing that really worked to reduce CPU usage acceptably was to turn of homotopy_class_planning.
        - max_number_classes - set to 2 for some CPU usage reduction
        - enable_homotopy_class_planning - set to false for drastic CPU usage reduction
        
- Navigation Tuning Guides - Both of these are very helpful in verifying that all the lower level systems are working effectively before trying to run/debug more complex behaviors. They also address costmap and global planner tuning.
    - [Official ROS wiki guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)
    - [Another tuning guide](http://www.zkytony.com/documents/navguide.pdf) - goes more in depth into some of the parts that the ROS wiki guide skims over
    
## Robust Controller Details
The robust controller was built using [smach](http://wiki.ros.org/smach) which is a Python library that handles task-level state machines. This makes it easy to add new states, change existing states, or completely modify the task flow with only a few lines of code. If you want to understand and/or modify parts of the robust_controller code, I suggest you start with and overview of the [smach concepts](http://wiki.ros.org/smach/Documentation) and then look at some of the [tutorials](http://wiki.ros.org/smach/Tutorials).

At it's core, the robust_controller is a state machine that consists of serveral states that are wrappers for move_base, but with slight modifications to the goals and results depending on which state robust_controller is in. The robust_controller state machine is then wrapped in an actionlib interface as described in the [smach actionlib wrapper tutorial](http://wiki.ros.org/smach/Tutorials/Wrapping%20a%20SMACH%20Container%20With%20actionlib).

Due to some time constraints, this code did not turn out to be the most user-friendly code for modification in the future so here are some tips if you are trying to modify the behavior of robust_controller:
- You can change HOME_POSE or NEAR_POSE in robust_controller.py as they are simply defined as constants (but should really be ROS parameters)
- You can change parameters for how many recovery attempts should be made, how often to save previous goals, and a tolerance for what is considered a previous goal in the code for UserGoalState in robust_controller.py. Again, these should be ROS parameters but that did not get implimented.
- The GOAL_NEAR, REVERT_POSE, and GO_HOME states call various versions of move_base_states.MoveBaseState
- The GO_HOME state calls MoveBaseStatewithListandResult() which has some special modifications to allow it to return actionlib results (tutorial [here](http://wiki.ros.org/smach/Tutorials/SimpleActionState)). Ideally something like this state could become the general state to call move_base and then the return values can just be handled seperately for each state. But as it is, it is just a messy separate state.
- I tried to be consistent with data/state/outcome naming conventions, as specified at the top of robust_controller.py, since smach requires so many different names for things
- smach can handle preemption of states and actions, however this was never specifically implimented or tested so it may or may not work in the current version of the code.

Unforunately, since rviz sends goals directly to move_base through its interface, it cannot be used to send goals to robust_controller (though I'm sure a plugin could be written to fix that). So instead we include a simple test script called controller_test.py that will send several goals to robust_controller. You can modify these goals for your particular map and then put obstacles in the robot's path and see how robust_controller performs.

## Cartographer Installation Notes
Cartograhper can sometimes be a bit of a challenge to install and run properly. Here are some tips that will hopefully be useful:
- You may install Cartographer in it's own workspace as shown in the installation instructions or, if you know what you are doing, you can install it in your current catkin workspace. If you install in a separate workspace, you may need to add a new source command to your .bashrc file. It will look something like `source ~/path/to/cartographer/install_isolated/setup.bash`.
- If you run into trouble later with sourcing from multiple workspaces, see [this thread](http://answers.ros.org/question/205976/sourcing-from-multiple-workspaces/) for tips on resolving this. The key is to make sure you build and source one workspace at a time since each source command overlays on the previous one.
- Since Cartographer is still under development, it is possible that future updates will cause incompatability with this code. If Cartograhper installs okay, but throws errors when being launched with one of the launch files from this repository, this is likely the case. There are two main reasons these errors would occur:
    1. The Cartographer team may have changed the structure of the parameter file - this will likely result in errors being printed about lua_parameter_dictionary and nil vlaues. Since we have a custom parameter file for Cartographer (in robust_navigation/param/), if the required structure of the file is changed, our parameter file will be incorrect. A simple fix for this is to look for recent changes to the [original parameter file](https://github.com/googlecartographer/cartographer_turtlebot/blob/master/cartographer_turtlebot/configuration_files/turtlebot_urg_lidar_2d.lua).
    2. The [Cartographer ROS API](https://google-cartographer-ros.readthedocs.io/en/latest/ros_api.html) may have changed - this might result in errors about ROS topics or services, or possible nothing happening at all. You can use the [roswtf](http://wiki.ros.org/roswtf) command to check that all of the topics are connected properly. If they aren't, you can [remap](http://wiki.ros.org/roslaunch/XML/remap) topics as necesary to connect everything up.

## Map Labelling Notes
The map labelling service has several things hard coded that are useful to be aware of:
- In the launch file, the path to the label file must be the global path. There is probably a ROS-like way to use $(find my_package) with it but using $(find ) as a parameter does not work so it is the global path for now
- The serivces expect to have the robot's pose published on /amcl_pose. To change the topic you currently need to modify the source code. This should really be changed to be a ROS paramter or arguement
- The robust_controller packages is hard coded in to handle receiving motion goals. The source code can be modified to switch it over to move_base fairly easily since both robust_controller and move_base take the same goal.

## Hector Navigation Notes
Since the hector_exploration package [does not integrate well with move_base](http://answers.ros.org/question/100136/hector-exploration-with-move-base/), hector_exploration_controller takes the plan from hector_exploration_planner and turns it into a goal for move_base. However, the default implimentation had a lot of problem so we had to modify hector_exploration_controller to try to mitigate the crashes and segfaults that would happen when the planner didn't return a path.

The custom implimentation does several different things to reduce crashes and help the robot continue to move even when the planner isn't giving useful information.
- If there is no plan returned, we try again and if there continues to be no plan, we try some basic recovery behaviors like turning and moving slightly forward to try and get the planner to return a plan
- If we do get a plan, we try to execute it by taking the last waypoint in the plan and sending that to move_base as the goal.
- Sometimes the planner returns a plan, but due to the interface with move_base, it doesn't result in the robot moving (we think because the goal for move_base ends up being the current location). When this happens a couple times in a row, we cut the plan into quarters and try to navigate to each waypoint along the way before requesting a new plan.

Even with all of this, the planner is often pretty slow and CPU intensive. Since hector_exploration was written for search and rescue applications, the planner is much more thorough and details than we really care about for indoor room/hallway mapping.

Additionally, as mentioned in the [Building Maps](#building-maps) section, there is a bug with the initial startup of hector_exploration. We have tried to mitigate the issue by sending a command for the robot to move forward slightly to initalize the map before it starts trying to call the planner, but this doesn't always work and so the robot sometimes needs to be rotated or pushed by hand slightly to get things initalized.

## Robust Navigation Torture Test
In order to test the robust navigation capabilities of our robot, we designed a simple torture test that randomly goes to many different locations in the map until we stop it or it runs out of battery. This lets the robot move about unsupervised in the real world and see how it performs. 

If you want to run your own version of the torture test included in robust_navigation, follow the steps below:
1. Build a map of the environment (See [Mapping section](#building-maps) above)
2. Use the [map_labelling services](#semantic-map-labelling) to create as many labels as you would like around the environment
3. Update the map_labelling launch file and torture_test launch file with correct label and map files
4. Edit robust_navigation/src/torture_test_controller.py and change the locations list at the top of the file to specify the label names
5. Set the robot at (0,0,0) in the map or edit amcl.launch to start at the correct initial location
6. Run `roslaunch robust_navigation torture_test.launch`
7. Visualize with `roslaunch robust_navigation view_navigation.launch`





