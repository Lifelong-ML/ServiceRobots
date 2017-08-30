/**************************************************************
  
  @file autodock.cc
  @author Obi Asinugo (GRASP-Lifelong ML Lab)
  @date 07/21/17
  @version 1.0 

  @brief Main file for turtlebot autodocking environment. 

**************************************************************/ 


#include <rl_env/autodock.hh> 
#include <cmath> 

using namespace std; 

/**
*   @brief Default constructor for AutoDockBot
*   
*   @param steps, the total number of steps taken before trial failure 
*   @return void 
*/
AutoDockBot::AutoDockBot(int steps){
  total_steps = steps;

  position.assign(2, 0.0); 

  get_data();

  position[0] = get_receiving_sensor();
  position[1] = get_orientation(); 
  cout << "AutoDockBot has started learning environment..." << endl;  
};


/**
*   @brief Destructor for AutoDockBot. 
* 
*   @return void 
*/
AutoDockBot::~AutoDockBot(){ 

};


/**
*   @brief Prints the IR data. 
*   
*   @return void 
*/ 
void AutoDockBot::print_ir_data(){
  string ostr_top; 
  string ostr_bot;

  ostr_top.append("[far]:"); 
  ostr_bot.append("[near]:"); 

  // Create table of IR data. 
  for(int i = 0; i < 3; i++){
    if(sensor_data[i] == 0) continue; 

    // Decode incoming IR data. 
    int top = sensor_data[i] >> 3;
    int bot = sensor_data[i];

    if(top & 2){ ostr_top.append(" L"); } else { ostr_top.append(" ~"); }
    if(top & 1){ ostr_top.append("|C"); } else { ostr_top.append("|~"); }
    if(top & 4){ ostr_top.append("|R"); } else { ostr_top.append("|~"); }
    if(bot & 1){ ostr_bot.append(" L"); } else { ostr_bot.append(" ~"); }
    if(bot & 2){ ostr_bot.append("|C"); } else { ostr_bot.append("|~"); }
    if(bot & 4){ ostr_bot.append("|R"); } else { ostr_bot.append("|~"); }
  }

  cout << ostr_top << endl; 
  cout << ostr_bot << endl;

  cout << "---------------------------" << endl; 
}


/**
*   @brief Gets IR data transmitted by docking station.
*  
*   @param ir_data, DockInfrared object containing IR data. 
*   @return void  
*/
void AutoDockBot::get_ir_data(const kobuki_msgs::DockInfraRed& ir_data){ 

  // Print IR data for each sensor. 
  for(int i = 0; i < 3; i++){
    sensor_data[i] = ir_data.data[i];
    cout << i << ": " << sensor_data[i] << endl;
  }

  cout << endl; 

  print_ir_data(); // DEBUGGING: print out IR data 

  // If no signal is received, set all values to IR info values to false.
  if(sensor_data[0] == 0 && sensor_data[1] == 0 && sensor_data[2] == 0){
    ir_info[0][0] = false; 
    ir_info[0][1] = false; 
    ir_info[0][2] = false; 
    ir_info[1][0] = false; 
    ir_info[1][1] = false; 
    ir_info[1][2] = false; 
  }
  else{
      // Update IR data matrix. 
      for(int i = 0; i < 3; i++){
        if(sensor_data[i] == 0) continue; 

        int top = sensor_data[i] >> 3;
        int bot = sensor_data[i];

        if(top & 2){ ir_info[0][0] = true; } else { ir_info[0][0] = false; }
        if(top & 1){ ir_info[0][1] = true; } else { ir_info[0][1] = false; }
        if(top & 4){ ir_info[0][2] = true; } else { ir_info[0][2] = false; }
        if(bot & 1){ ir_info[1][0] = true; } else { ir_info[1][0] = false; }
        if(bot & 2){ ir_info[1][1] = true; } else { ir_info[1][1] = false; }
        if(bot & 4){ ir_info[1][2] = true; } else { ir_info[1][2] = false; }
      }
  }
}

/**
*   @brief Gets sensor information for determining charging state. 
* 
*   @param core_data, SensorState object containing charging state of the bot.
*   @return void 
*/
void AutoDockBot::get_core_data(const kobuki_msgs::SensorState& core_data){
  charge_state = core_data.charger; // store charging state 

  // cout << "DEBUG CORE DATA CHARGE: " << charge_state << endl; 

  if(charge_state == 0){
    cout << "DISCHARGING" << endl; 
  }
  else{
    cout << "CHARGING" << endl; 
  }
}


/**
*   @brief Handles getting and updating IR info and charging state. 
* 
*   @return void 
*/
void AutoDockBot::get_data(){

  ros::NodeHandle nh; 

  ros::Rate rate(1000);  
  ros::Time start;

  // Subscribers
  ros::Subscriber dock_ir = nh.subscribe("/mobile_base/sensors/dock_ir", 1000, &AutoDockBot::get_ir_data, this);
  ros::Subscriber core = nh.subscribe("/mobile_base/sensors/core", 1000, &AutoDockBot::get_core_data, this);

  float duration = 0.2; 
  start = ros::Time::now(); 
  while(ros::Time::now() - start < ros::Duration(duration)){
    ros::spinOnce(); 
    rate.sleep(); 
  }
}


/**
*   @brief Gets the total number of actions available in this environment. 
*
*   @return int, total number of actions.
*/
int AutoDockBot::getNumActions(){
	return 5; 
}

/** 
*   @brief Provides access to the current sensation that the environment gives 
        to the agent. 
* 
*   @return vector<float>, vector containing position vector data. 
*/
const std::vector<float>& AutoDockBot::sensation() const { 
	return position; 
}

/**
*   @brief Gets the minimum and maximum of the features in the environment. 
* 
*   @param minFeat, vector of minimum features 
*   @param maxFeat, vector of maximum features 
*   @return void 
*/
void AutoDockBot::getMinMaxFeatures(std::vector<float> *minFeat,
                                 std::vector<float> *maxFeat){
  
  minFeat->resize(position.size(), 0.0);
  maxFeat->resize(position.size(), 10.0);

  (*maxFeat)[0] = 5.0;

}

/**
*   @brief Gets the minimum and maximum one-step reward in the domain. 
*
*   @param minR, pointer to the minimum reward 
*   @param maxR, pointer to the maximum reward 
*   @return void 
*/
void AutoDockBot::getMinMaxReward(float *minR, float *maxR){
	*minR = -100; 
	*maxR = 100; 
}

/**
*   @brief Determines whether the environment has reached a terminal state--   
        in this case, if the robot successfully docks. 
*
*   @return bool, true if bot succesfully docks; false otherwise  
*/
bool AutoDockBot::terminal() const{
  cout << "Charge State: " << charge_state << endl; 

  if(charge_state == 0){
    return false; 
  }
  else{
    return true; 
  }  
}


/**
*   @brief Determines reward for a particular state-action pair. 
*
*   @param negative_reward, true if negative reward should be awarded; false if
        otherwise. 
*   @return float, reward amount. 
*/
float AutoDockBot::reward(bool negative_reward){
	if(negative_reward){
		return -100; 
	}
	else{
		if(terminal()){
			return 100; 
		}
		else{
			return 10; 
		}
	}
}

/**
*   @brief Gets the sensor that is receiving IR data. 
*
*   @return int, sensor that is receiving IR data. 
*/ 
int AutoDockBot::get_receiving_sensor(){
  if(sensor_data[0] != 0){
    return LEFT_SENSOR; 
  }
  else if(sensor_data[1] != 0){
    return CENTER_SENSOR; 
  }
  else if(sensor_data[2] != 0){
    return RIGHT_SENSOR;
  }
  else{
      return NONE;
  }
}

/**
*   @brief Gets the orientation of the bot relative to the docking station.
*
*   @return int, orientation of the bot relative to the docking station. 
*/
int AutoDockBot::get_orientation(){
  if(ir_info[1][1]){
    cout << "Orientation: NEAR CENTER" << endl; 
    return NEAR_CENTER; 
  }
  else if(ir_info[1][0]){
    cout << "Orientation: NEAR LEFT" << endl;
    return NEAR_LEFT; 
  }
  else if(ir_info[1][2]){
    cout << "Orientation: NEAR RIGHT" << endl;
    return NEAR_RIGHT; 
  }
  else if(ir_info[0][1]){
    cout << "Orientation: FAR CENTER" << endl;
    return FAR_CENTER; 
  }
  else if(ir_info[0][0]){
    cout << "Orientation: FAR LEFT" << endl;
    return FAR_LEFT; 
  }
  else if(ir_info[0][2]){
    cout << "Orientation: FAR RIGHT" << endl;
    return FAR_RIGHT; 
  }

  cout << "Orientation: BLIND" << endl; 
  return BLIND;
}

/**
*   @brief Updates the position vector based on action. 
*
*   @param action, action taken by bot 
*   @return void 
*/
void AutoDockBot::update_position(int action){ 
      moveBot(action);
      get_data(); 
      position[0] = get_receiving_sensor();
      position[1] = get_orientation(); 
}

/**
*   @brief Resets the internal state of the environment according to some 
        initial state distribution. 
*
*   @return void 
*/
void AutoDockBot::reset(){
  // clear history of actions 
  action_history.clear(); 

  position.clear(); 
  position.resize(2, 0.0); 

  get_data(); 

  cout << "RESET POSITION: " << endl; 

  position[0] = get_receiving_sensor();
  position[1] = get_orientation(); 
}

/**
*   @brief Bot moves in a particular directino based on action. 
*
*   @param action, action taken by bot. 
*   @return void
*/
void AutoDockBot::moveBot(int action){
  // Create publisher to driving topic. 
  ros::NodeHandle nh; 
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1000);
  ros::Rate rate(10);  
  ros::Time start; 

  // Create a message. 
  geometry_msgs::Twist move; 
  float duration = 0.5; 

  // determine movement based on given action. 
  switch(action){
    case FORWARD:
      cout << "move FORWARD" << endl; 
      start = ros::Time::now(); 
      while(ros::Time::now() - start < ros::Duration(duration)){
        // move forward 
        move.linear.x = 0.075; 
        move.angular.z = 0; 
        pub.publish(move); 

        ros::spinOnce(); 
        rate.sleep(); 
      }
      break; 
    case BACKWARD:
      cout << "move BACKWARD" << endl; 
      start = ros::Time::now(); 
      while(ros::Time::now() - start < ros::Duration(duration)){
        // move backward         
        move.linear.x = -0.075; 
        move.angular.z = 0; 
        pub.publish(move); 

        ros::spinOnce(); 
        rate.sleep(); 
      }
      break;  
    case TURN_RIGHT: 
      cout << "move TURN RIGHT" << endl; 
      start = ros::Time::now(); 
      while(ros::Time::now() - start < ros::Duration(duration)){
        // turn right  
        move.linear.x = 0; 
        move.angular.z = -1.25; 
        pub.publish(move); 

        ros::spinOnce(); 
        rate.sleep(); 
      }
      while(ros::Time::now() - start < ros::Duration(duration)){
        // move forward 
        move.linear.x = 0.075; 
        move.angular.z = 0; 
        pub.publish(move); 

        ros::spinOnce(); 
        rate.sleep(); 
      }
      break; 
    case TURN_LEFT: 
      cout << "move TURN LEFT" << endl; 
      start = ros::Time::now(); 
      while(ros::Time::now() - start < ros::Duration(duration)){
        // turn left  
        move.linear.x = 0; 
        move.angular.z = 1.25; 
        pub.publish(move); 

        ros::spinOnce(); 
        rate.sleep(); 
      }
      while(ros::Time::now() - start < ros::Duration(duration)){
        // move forward 
        move.linear.x = 0.075; 
        move.angular.z = 0; 
        pub.publish(move); 

        ros::spinOnce(); 
        rate.sleep(); 
      }
      break;
    case SCAN: 
      cout << "move SCAN" << endl; 
      start = ros::Time::now(); 
      while(ros::Time::now() - start < ros::Duration(0.25)){
        // turn left  
        move.linear.x = 0; 
        move.angular.z = 1.25; 
        pub.publish(move); 

        ros::spinOnce(); 
        rate.sleep(); 
      }
      break;
  }
}

/** 
*   @brief Update reward value based on previous sensor and orientation info. 
* 
*   @param prev_sensor, previous sensor receiving data. 
*   @param prev_orientation, previous orientation of bot. 
*   @return int, reward amount. 
*/
int AutoDockBot::sensor_reward(float prev_sensor, float prev_orientation, int action){
  if((position[0] == LEFT_SENSOR || position[0] == RIGHT_SENSOR) && (position[0] != prev_sensor)){
    return reward(true);
  }
  else{
    if((prev_orientation == BLIND && action != SCAN) || (action == SCAN && prev_orientation != BLIND)){
      return reward(true);
    }
    else if(position[1] == prev_orientation){
      return reward(false);
    }
    else if(position[1] == NEAR_CENTER || position[1] == FAR_CENTER){
      return reward(false); 
    }
    else{
      return reward(true);
    }
  }
}

/** 
*   @brief Allows an agent to affect its environment based on the action.
* 
*   @param action, action taken by bot. 
*   @return float, reward amount. 
*/
float AutoDockBot::apply(int action) {
  int act_used = action; 

  action_history.push_back(action);
  
  float prev_sensor, prev_orientation; 

  // store previous positions
  prev_sensor = position[0]; 
  prev_orientation = position[1]; 

  // update position based on action 
  update_position(action); 

  return sensor_reward(prev_sensor, prev_orientation, action);
}

/**
*   @brief Create an experience tuple for the given state-action. 
* 
*   @param s0, recent sensor receiving IR data. 
*   @param s1, orientation of bot relative to station. 
*   @return experience, experience tuple for the given state-action. 
*/
experience AutoDockBot::getExp(float s0, float s1, int a){
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

/** 
*   @brief Gets seeding experiences for agent. 
* 
*   @return vector<experience>, vector of seeding experiences. 
*/
std::vector<experience> AutoDockBot::getSeedings() {

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

