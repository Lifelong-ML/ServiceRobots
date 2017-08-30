/* Header file for 1D grid world */ 
#ifndef _GRIDWORLD2_H_
#define _GRIDWORLD2_H_

#include <ros/ros.h> 
#include <geometry_msgs/Twist.h> 
#include <rl_common/Random.h> 
#include <rl_common/core.hh> 
#include <iostream> 
#include <vector> 
#include <deque> 

class Gridworld2: public Environment { 
	public:
		// TO-DO: 
			// 1. add destructor
		Gridworld2(unsigned height, unsigned width, unsigned target_row, unsigned target_col); 

		virtual ~Gridworld2(); 
		
		unsigned height() { return h; }
		unsigned width() { return w; }
		void update_position(int action); 
		bool wall(int action); 
		void set_target(int row, int column){
			target[0] = row; 
			target[1] = column; 
		}
    void moveBot(int action); 

	  virtual const std::vector<float> &sensation() const;
	  virtual float apply(int action);

	  virtual bool terminal() const;
	  virtual void reset();

	  virtual int getNumActions();
	  virtual void getMinMaxFeatures(std::vector<float> *minFeat, std::vector<float> *maxFeat);
	  virtual void getMinMaxReward(float* minR, float* maxR);

	  /** Create an experience tuple for the given state-action. */
	  experience getExp(float s0, float s1, int a);

  	  virtual std::vector<experience> getSeedings();
  	  float reward(bool negative_reward); 

	private: 
		unsigned h; 
		unsigned w; 
		int** grid;
		std::deque<int> action_history; 
		std::vector<float> position; 
		std::vector<float> target; 
//    ros::Publisher out_move; 

	protected: 
		enum grid_action {NORTH, SOUTH, EAST, WEST}; 
}; 

#endif 

