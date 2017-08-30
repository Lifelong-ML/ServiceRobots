#include <rl_env/gridworld2.hh> 
#include <cmath> 

using namespace std; 

Gridworld2::Gridworld2(unsigned height, unsigned width, unsigned target_row, unsigned target_col){
			h = height; 
			w = width; 

			 // grid = new int[height][width]; 
			position.assign(2, 0.0);
			target.assign(2, 0.0); 

			//cout << "test 1" << endl; 

			target[0] = target_row; 
			target[1] = target_col;  

			//cout << "test 2" << endl; 

};

Gridworld2::~Gridworld2(){ 
			delete[] grid; 
};

int Gridworld2::getNumActions(){
	return 4; 
}

const std::vector<float>& Gridworld2::sensation() const { 
	return position; 
}


void Gridworld2::getMinMaxFeatures(std::vector<float> *minFeat,
                                 std::vector<float> *maxFeat){
  
  minFeat->resize(position.size(), 0.0);
  maxFeat->resize(position.size(), 10.0);

  (*maxFeat)[0] = 5.0;

}

void Gridworld2::getMinMaxReward(float *minR, float *maxR){
	*minR = -100; 
	*maxR = 100; 
}

bool Gridworld2::wall(int action){
	switch(action){
		case NORTH: 
			return position[0] - 1 < 0; 

		case SOUTH: 
			return position[0] + 1 > h-1; 

		case EAST: 
			return position[1] + 1 > w-1; 

		case WEST: 
			return position[1] - 1 < 0; 

		default: 
			return true; 
	}
}

bool Gridworld2::terminal() const{
	return (position[0] == target[0]) && (position[1] == target[1]); 
}


float Gridworld2::reward(bool negative_reward){
	if(negative_reward){
		return -100; 
	}
	else{
		if(terminal()){
			return 100; 
		}
		else{
			return 0; 
		}
	}
}

void Gridworld2::update_position(int action){
  switch(action){
		case NORTH: 
			position[0] = position[0] - 1; 
      moveBot(action); 
			break; 
		case SOUTH: 
			position[0] = position[0] + 1; 
      moveBot(action);
			break; 
		case EAST: 
			position[1] = position[1] + 1; 
      moveBot(action);
			break; 
		case WEST: 
			position[1] = position[1] - 1; 
      moveBot(action);
			break; 
		default: 
			break;  
	}
}

void Gridworld2::moveBot(int action){
  ros::NodeHandle nh; 
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1000);
  ros::Rate rate(10);  
  ros::Time start; 

 // cout << "here" << endl; 

  // Create a message. 
  geometry_msgs::Twist move; 
  
  float duration = 0.5; 
  switch(action){
    case NORTH:
      cout << "move NORTH" << endl; 
      start = ros::Time::now(); 
      while(ros::Time::now() - start < ros::Duration(duration)){
        // move forward 
        move.linear.x = -0.1; 
        move.angular.z = 0; 
        pub.publish(move); 

        ros::spinOnce(); 
        rate.sleep(); 
      }
      break; 
    case SOUTH:
      cout << "move SOUTH" << endl; 
      start = ros::Time::now(); 
      while(ros::Time::now() - start < ros::Duration(duration)){
        // move forward 
        move.linear.x = 0.1; 
        move.angular.z = 0; 
        pub.publish(move); 

        ros::spinOnce(); 
        rate.sleep(); 
      }
      break;  
    case EAST: 
      cout << "move EAST" << endl; 
      start = ros::Time::now(); 
      while(ros::Time::now() - start < ros::Duration(duration)){
        // turn right  
        move.linear.x = 0; 
        move.angular.z = -2.25; 
        pub.publish(move); 

        ros::spinOnce(); 
        rate.sleep(); 
      }
      while(ros::Time::now() - start < ros::Duration(duration)){
        // move forward 
        move.linear.x = 0.1; 
        move.angular.z = 0; 
        pub.publish(move); 

        ros::spinOnce(); 
        rate.sleep(); 
      }
      break; 
    case WEST: 
      cout << "move WEST" << endl; 
      start = ros::Time::now(); 
      while(ros::Time::now() - start < ros::Duration(duration)){
        // turn right  
        move.linear.x = 0; 
        move.angular.z = 2.25; 
        pub.publish(move); 

        ros::spinOnce(); 
        rate.sleep(); 
      }
      while(ros::Time::now() - start < ros::Duration(duration)){
        // move forward 
        move.linear.x = 0.1; 
        move.angular.z = 0; 
        pub.publish(move); 

        ros::spinOnce(); 
        rate.sleep(); 
      }
      break;
  }
}

void Gridworld2::reset(){
	// clear history of actions 
	action_history.clear(); 

	// reset position 
	//position = new int[2];
	position.clear(); 
	position.resize(2, 0.0); 
}

float Gridworld2::apply(int action) {
	int act_used = action; 

	action_history.push_back(action);
  
  float prev_x_pos, prev_y_pos; 

    switch(action) {
	    case NORTH:
	      if (wall(action)){
	      	// return negative reward; do not update position 
	      	return reward(true);  
	      }

        // store previous positions
       prev_x_pos = position[0]; 
       prev_y_pos = position[1]; 

	     update_position(action); 

        if((target[0] - prev_x_pos) < (target[0] - position[0])){
          // return negative reward 
          return reward(true); 
        }
        else if((target[1] - prev_y_pos) < (target[1] - position[1])){
          // return negative reward
          return reward(true); 
        }

        return reward(false); 
	    case SOUTH:
	      if (wall(action)){
	      	// return negative reward; do not update position 
	      	return reward(true);  
	      }

        // store previous positions
       prev_x_pos = position[0]; 
       prev_y_pos = position[1]; 

	      update_position(action); 

        if((target[0] - prev_x_pos) < (target[0] - position[0])){
          // return negative reward 
          return reward(true); 
        }
        else if((target[1] - prev_y_pos) < (target[1] - position[1])){
          // return negative reward
          return reward(true); 
        }

        return reward(false); 

	    case EAST:
	      if (wall(action)){
	      	// return negative reward; do not update position 
	      	return reward(true);  
	      }

        // store previous positions
        prev_x_pos = position[0]; 
        prev_y_pos = position[1]; 

	      update_position(action); 

        if((target[0] - prev_x_pos) < (target[0] - position[0])){
          // return negative reward 
          return reward(true); 
        }
        else if((target[1] - prev_y_pos) < (target[1] - position[1])){
          // return negative reward
          return reward(true); 
        }

        return reward(false); 

	    case WEST:
	      if (wall(action)){
	      	// return negative reward; do not update position 
	      	return reward(true);  
	      }

        // store previous positions
        prev_x_pos = position[0]; 
        prev_y_pos = position[1]; 

	      update_position(action); 

        if((target[0] - prev_x_pos) < (target[0] - position[0])){
          // return negative reward 
          return reward(true); 
        }
        else if((target[1] - prev_y_pos) < (target[1] - position[1])){
          // return negative reward
          return reward(true); 
        }

        return reward(false); 
    }

    std::cerr << "Unreachable point reached in Gridworld2::apply!!!\n";
 
  return 0; 
}



experience Gridworld2::getExp(float s0, float s1, int a){
  experience e;

  e.s.resize(2, 0.0);
  e.next.resize(2, 0.0);

  e.act = a;
  e.s = sensation();
  e.reward = apply(e.act);

  e.terminal = terminal();
  e.next = sensation();

  reset(); 

  return e; 
}

std::vector<experience> Gridworld2::getSeedings() {

  // return seedings
  std::vector<experience> seeds;

  //cout << "test 3" << endl; 

  // single seed of terminal state
  if(action_history.empty()){
  	seeds.push_back(getExp(position[0], position[1], 0));
  }
  else{
  	seeds.push_back(getExp(position[0], position[1], action_history.back()));
  }

  action_history.clear();
  
  reset();

  return seeds;

}

