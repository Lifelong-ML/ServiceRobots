//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#include <ros/ros.h>
#include <hector_path_follower/hector_path_follower.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <math.h>
#include <geometry_msgs/Twist.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 

class SimpleExplorationController
{
public:
  SimpleExplorationController()
  {

    exploration_plan_service_client_ = nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>("get_exploration_path");

    path_follower_.initialize(&tfl_);

    mb_ac = new MoveBaseClient("move_base", true); 

    while(!mb_ac->waitForServer(ros::Duration(5.0))) { 
  ROS_INFO("Waiting for the move_base action server to come up"); 
    }


  }

  ~SimpleExplorationController() 
  {
    delete mb_ac; 
  }

  ros::NodeHandle nh; 

  ros::ServiceClient exploration_plan_service_client_;

  tf::TransformListener tfl_;

  pose_follower::HectorPathFollower path_follower_;

  MoveBaseClient *mb_ac; 

};


int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  SimpleExplorationController ec;

  int failure_count = 0;
  bool need_new_plan = true;
  int pose_num = 0;
  int plan_size = 0;
  int old_plan_size = 0;
  int num_old_plans = 0;
  hector_nav_msgs::GetRobotTrajectory srv_exploration_plan;
  ros::Rate r(2);
  

//Avoid footprint not set bug
  //https://github.com/ros-planning/navigation/issues/327
  move_base_msgs::MoveBaseGoal goal; 
  goal.target_pose.header.frame_id = "/base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.2;
  goal.target_pose.pose.orientation.w = 1;
  ROS_INFO("Moving forward slightly to fix footprint");
  ec.mb_ac->sendGoal(goal); 
  ec.mb_ac->waitForResult(ros::Duration(10.0));  
  ec.mb_ac->cancelGoal(); 
  ros::Duration(3).sleep();


  while(ros::ok()) { 
    
     ros::spinOnce(); 

     move_base_msgs::MoveBaseGoal goal; 
     
     //do we need a new plan?
     if (need_new_plan == true)
     {
  	ROS_INFO("Generating plan"); 
        ec.exploration_plan_service_client_.call(srv_exploration_plan);
	plan_size = srv_exploration_plan.response.trajectory.poses.size();
	pose_num = 0;

        //check to see if we have a plan
        if (plan_size > 1){
          ROS_INFO("Generated exploration path with %u poses", (unsigned int) plan_size);
          ec.path_follower_.setPlan(srv_exploration_plan.response.trajectory.poses);
 	  //try to ensure we aren't stuck    	  
	  if (plan_size == old_plan_size)
	  {
		num_old_plans++;
		if (num_old_plans > 4)
		{
			pose_num = plan_size/4;
			need_new_plan = false;
		}
		else
		{
			pose_num = plan_size-1;
			need_new_plan = true;
		}
		
          }
	  else
	  {
	        pose_num = plan_size-1;
		need_new_plan = true;
		num_old_plans = 0;
	  }

          goal.target_pose.header.frame_id = "/map"; 
          goal.target_pose.header.stamp = ros::Time::now(); 
          goal.target_pose.pose = srv_exploration_plan.response.trajectory.poses[pose_num].pose; 
	  failure_count = 0;
	  old_plan_size = plan_size;
	  ROS_INFO("Executing pose %u of plan", (unsigned int) pose_num);
	
        }
	//if we don't get back a plan, try recovery behaviors
	else{
          ROS_WARN("Service call for exploration service failed");
          failure_count++;

	   if(failure_count < 2){
             goal.target_pose.header.frame_id = "/base_link";
             goal.target_pose.header.stamp = ros::Time::now();
	    }
	    else if(failure_count >= 2 && failure_count < 4){
             goal.target_pose.header.frame_id = "/base_link";
             goal.target_pose.header.stamp = ros::Time::now();
	     float angle = 45*M_PI/180; 
             goal.target_pose.pose.orientation.z = sin(angle/2);
	     goal.target_pose.pose.orientation.w = cos(angle/2);
             ROS_INFO("Attempting to recover via rotation");
            }
	   else if(failure_count >= 4 && failure_count < 6){
             goal.target_pose.header.frame_id = "/base_link";
             goal.target_pose.header.stamp = ros::Time::now();
             goal.target_pose.pose.position.x = 0.5;
             goal.target_pose.pose.orientation.w = 1;
             ROS_INFO("Attempting to recover via movement");
	   }
	   else{
             goal.target_pose.header.frame_id = "/base_link";
             goal.target_pose.header.stamp = ros::Time::now();
	     ROS_ERROR("RECOVERY FAILURE");
	   }		

        }
      }
      //if we already have a plan just get a new pose
      else
      {
	pose_num += plan_size/4;
	if (pose_num >= (plan_size-1))
	{
	  need_new_plan = true;
	  pose_num = plan_size-1;
	}
	ROS_INFO("Executing pose %u of plan", (unsigned int) pose_num);
        goal.target_pose.header.frame_id = "/map"; 
        goal.target_pose.header.stamp = ros::Time::now(); 
        goal.target_pose.pose = srv_exploration_plan.response.trajectory.poses[pose_num].pose; 
      }

      //Send goal to base
      ec.mb_ac->sendGoal(goal); 
      ec.mb_ac->waitForResult(ros::Duration(10.0));  
      ec.mb_ac->cancelGoal(); 
      r.sleep(); 
  }

  return 0;
}
