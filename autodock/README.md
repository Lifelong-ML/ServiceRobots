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

3. Move the contents in the autdock directory to your catkin workspace. For information on setting up a catkin workspace, refer to the instructions [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
4. Run catkin_make at the top level of your workspace to compile all of the code and messages. 
5. Source the setup script with `source devel/setup.bash`. You can add `source ~/path/to/workspace/devel/setup.bash` to your .bashrc file to auto-source this workspace in the future.
6. Try and run some simple ROS commands: 
	
	`rospack find rl_agent`  
	`rosmake rl_agent`

	These should compile without fault. 

# Run Instructions 

## Running an experiment 
In order for the agent and environment to communicate, one must bring up the TurtleBot. So start first by running:
 
`roslaunch turtlebot_bringup minimal.launch`

We can now start an agent from the rl_agent package. For our experiments, we utilized the Q-Learning agent, but one can opt to use other agents, which can be found in more detail [here](http://wiki.ros.org/rl_agent). Make sure you've sourced the setup script. To start the Q-Learning agent, enter the following command in a new terminal: 

`rosrun rl_agent agent --agent qlearner --filename test`

In this case, we've selected the Q-Learning algorithm with the '--agent qlearner' option and decided to save our policy under the filename 'test' with the '--filename test' option.

Once the agent is running, we can start our environment. Run the Autodock environment in a new tab/terminal by enter the following command: 

`rosrun rl_env env --env autodock`

We were able to select and start-up the Autodock environment with the '--env autodock' option. 

The experiment should now be running. If you switch back to the agent tab, the sum of the rewards for each epsiode will appear. Press `Ctrl+C` in the agent tab to end the experiment. The terminal should print out the policy (Q-values) for the experiment. 






