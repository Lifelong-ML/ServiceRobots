# Overview
This collection of packages aims to provide a robust autodocking procedure for a TurtleBot 2.0. 

Five packages are provided in the src directory:
- rl_common:     Some files that are common to both agents and environments.
- rl_msgs:       Definitions of ROS messages for agents and envs to communicate.
- rl_agent:      A library of some RL agents (such as Q-Learning).
- rl_env:        A library of some RL environments.
- rl_experiment: Code to run some RL experiments without ROS message passing.

The core packages are provided by Todd Hester's reinforcement learning ROS stack (rl-texplore-pkg). Documentation and a tutorial for these packages can be found at these links, respectively:

http://www.ros.org/wiki/reinforcement_learning
http://www.ros.org/wiki/reinforcement_learning/Tutorials/Reinforcement%20Learning%20Tutorial 

To test and determine an optimal policy for autodocking, an Autodock environment was created within the [rl_env](http://wiki.ros.org/rl_env) package.

# Installation
1. Install ROS [Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) - Full Desktop installation recommended. 	
2. Clone this repository to your computer. 

	`git clone https://github.com/GRASP-ML/ServiceRobots.git`

3. Move the contents in the autdock directory to your catkin workspace. For information on setting up a catkin workspace, refer to the instructions [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
4. Run catkin_make at the top level of your workspace to compile all of the code and messages. 
5. Source the setup script with `source devel/setup.bash`. You can add `source ~/path/to/workspace/devel/setup.bash` to your .bashrc file to auto-source this workspace in the future.
6. Try and run some simple ROS commands: 
	
	`rospack find rl_agent`
	`rosmake rl_agent`

	These should compile without fault. 






