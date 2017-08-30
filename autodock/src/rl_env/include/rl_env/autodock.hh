/**************************************************************
	
	@file	autodock.hh
	@author Obi Asinugo (GRASP-Lifelong ML Lab)
	@date 07/21/17
	@version 1.0 

	@brief Header file for turtlebot autodocking environment. 

**************************************************************/ 
#ifndef _AUTODOCK_H
#define _AUTODOCK_H

#include <ros/ros.h> 
#include <string>
#include <geometry_msgs/Twist.h> 
#include <rl_common/Random.h> 
#include <rl_common/core.hh> 
#include <iostream> 
#include <vector> 
#include <deque> 
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/DockInfraRed.h>

/**
* @brief Interface for the autodocking environment, whose states can be 
	represented as vectors of floats and whose actions can be represented as ints. Class that maintains all the core function and data for autodocking 
* 		 a TurtleBot. 
*/

class AutoDockBot: public Environment { 
	public:
	  // Default constructor 	
	  AutoDockBot(int steps);

	  // Destructor  
	  virtual ~AutoDockBot(); 
	  
	  // Update position based on action.	
	  void update_position(int action); 

	  // Bot moves in a particular direction based on action.
	  void moveBot(int action); 

	  // Get IR and sensor information. 
	  void get_data(); 

	  // Get IR data.
	  void get_ir_data(const kobuki_msgs::DockInfraRed& ir_data); 

	  // Get sensor data. 
      void get_core_data(const kobuki_msgs::SensorState& core_data); 

      // Print IR data.
      void print_ir_data(); 

      // Update sensor data.
      void update_sensor_data();

      // Update reward value based on previous sensor, orientation info, and action .
      int sensor_reward(float prev_sensor, float prev_orientation, int action); 

      // Get sensor that is receiving IR data. 
      int get_receiving_sensor(); 

      // Get orientation of the bot relative to the docking station. 
      int get_orientation(); 

      // Provides access to the current sensation that the environment
      // gives to the agent. 
	  virtual const std::vector<float> &sensation() const;

	  // Allows an agent to affect its environment based on the action.
	  virtual float apply(int action);

	  // Determines whether the environment has reached a terminal state.
	  virtual bool terminal() const;

	  // Resets the internal state of the environment according to some initial state distribution. 
	  virtual void reset();

	  // Gets the total umber of actions available in this environment. 
	  virtual int getNumActions();

	  // Gets the minimum and maximum of the features in the environment. 
	  virtual void getMinMaxFeatures(std::vector<float> *minFeat, std::vector<float> *maxFeat);
	  
	  // Gets the minimum and maximum one-step reward in the domain. 
	  virtual void getMinMaxReward(float* minR, float* maxR);

	  // Create an experience tuple for the given state-action.
	  experience getExp(float s0, float s1, int a);

	  // Get seeding experiences for agent. 
  	  virtual std::vector<experience> getSeedings();

  	  // Determine reward for a particular state-action pair. 
  	  float reward(bool negative_reward); 

	private: 
		int total_steps;  // Total steps taken by bot. 
		int charge_state; // Charger state for docking.  
		std::deque<int> action_history; // Queue of previous actions taken.
		std::vector<float> position; // Vector holding the receiving sensor and orientation of the bot.
		bool ir_info[2][3]; // first row: far ir info (left, center, right)
							// second row: near ir info (left, center, right)
		int sensor_data[3]; // sensor receiving info (left, center, right) 

	protected: 
		enum actions {FORWARD, BACKWARD, TURN_RIGHT, TURN_LEFT, SCAN}; // actions taken by bot 
		enum sensor {LEFT_SENSOR, CENTER_SENSOR, RIGHT_SENSOR, NONE}; // IR sensors on the bot 
		enum positions {FAR_RIGHT, FAR_CENTER, FAR_LEFT, NEAR_RIGHT, NEAR_CENTER, NEAR_LEFT, BLIND}; // relative positions (orientation) of bot to docking station
}; 

#endif 


