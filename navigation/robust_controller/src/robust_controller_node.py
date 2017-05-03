#!/usr/bin/python

#==========================================================
# imports

import rospy
import smach, smach_ros
from geometry_msgs.msg import PoseStamped
import move_base_states as mb_states
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib
import math
from robust_controller.msg import RobustControllerAction,RobustControllerResult

"""
Naming conventions for FSM:
STATE_NAME
StateNameClass()
outcome_statename_*  -for states
succeeded  			 -for SMs
kStateMachineDataKey
sStateDataKey
"""

#========================================================
# States

#This state will need a custom move base client to do the more complex processing
class UserGoalState(smach.State):
	def __init__(self,rec_count=3,t_save=5,prev_dist=2):
		"""Initialize state and define class variables
			rec_count: number of times recovery can be called before aborting
			t_save: time in seconds between saving poses
			prev_dist: distance (m) from stuck pose for a previous pose to be considered valid
					Note - if there is no pose farther away than prev_dist, the first pose in the list
							will be chosen."""
							
		smach.State.__init__(self, 
                             outcomes=['outcome_movebase_arrived',
									   'outcome_movebase_failed',
									   'outcome_movebase_arrived_trouble',
									   'outcome_movebase_failed_too_many'],
                             input_keys=['sPose','sRecCounter_in','sResult'],
							 output_keys=['sRecCounter_out','sPrevPose','sResult'])

		self.rec_count = rec_count
		self.t_save = t_save
		self.PrevPoseList = []
		self.t0 = rospy.get_time()	
		self.prev_dist = prev_dist

	#Fuction to process feedback data from move base
	def mb_callback(self,feedback):

		#keep list of previous poses
		if rospy.get_time() - self.t0 > self.t_save:
			self.PrevPoseList.append(feedback.base_position)
			self.t0 = rospy.get_time()
		
	def choosePrevPose(self):
		""" Go through PrevPoseList and choose one pose to return """

		n = len(self.PrevPoseList)
		if n == 0:
			return PoseStamped() #if there has been no feedback, then just default to (0,0,0)
		else:
			cur_pose = self.PrevPoseList[-1] #estimate the current pose as the last feedback in list

		#loop backwards through list and find first pose that is farther away than self.prev_dist
		for i in xrange(n-1,0,-1):	
			test_pose = self.PrevPoseList[i]
			dist = math.sqrt((test_pose.pose.position.x - cur_pose.pose.position.x)**2 + (test_pose.pose.position.y - cur_pose.pose.position.y)**2)
			
			#return test pose if it is far enough away
			if dist > self.prev_dist:
				return test_pose

		#if nothing is far enough away just return the first pose in the list
		return self.PrevPoseList[0]
			

	def execute(self, userdata):
		rospy.loginfo('Executing state USER_GOAL')
			
		#create a move base client and send goal
		client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		client.wait_for_server() 
		#actionGoal = MoveBaseGoal()
		actionGoal = userdata.sPose

		#send goal and register feedback callback
		client.send_goal(actionGoal,feedback_cb=self.mb_callback)

		#wait for results
		client.wait_for_result()
		
		#determine if we succeeded or failed
		status = client.get_state()
		
		#Status 3 means success
		#might need to handle preepted case as well for if a new goal is given
		if status==3:
			if userdata.sRecCounter_in > 0:
				userdata.sResult.result = 2
				userdata.sRecCounter_out = 0
				return 'outcome_movebase_arrived_trouble'
			else:
				userdata.sResult.result = 1
				return 'outcome_movebase_arrived'
		else:

			#increment recovery counter
			userdata.sRecCounter_out = userdata.sRecCounter_in + 1
			#update userdata from feedback
			userdata.sPrevPose = self.choosePrevPose()

			if userdata.sRecCounter_in > self.rec_count:
				return 'outcome_movebase_failed_too_many'
			else:
				return 'outcome_movebase_failed'



#==================================================
#Construct SM

def main():

	rospy.init_node('robust_controller_server') 

	# Construct master state machine
	sm = smach.StateMachine(outcomes=[	'arrived',
									  	'arrived_with_trouble',
										'aborted_home',
										'aborted_stuck',
										'preempted'],
							input_keys = ['kuser_goal'],
							output_keys = ['kuser_result'])

	#Pose parameters
	HOME_POSE = [0,0,0] #this should be in the map coordinates
	NEAR_POSE = [-1,0,0] #this is in the local coordinates	

	#initialize pose for testing
	"""sm.userdata.kuser_goal = PoseStamped() #input pose
	sm.userdata.kuser_goal.header.frame_id = "map"
	sm.userdata.kuser_goal.header.stamp = rospy.get_rostime()
	sm.userdata.kuser_goal.pose.position.x = 1
	sm.userdata.kuser_goal.pose.position.y = -1
	sm.userdata.kuser_goal.pose.orientation.w = 1"""

	#initialize userdata
	sm.userdata.kPrevPose = PoseStamped() #previous 'good' pose
	sm.userdata.kRecCounter = 0 #counter for number of recovery attempts
	sm.userdata.kHomePose = HOME_POSE
	sm.userdata.kuser_result = RobustControllerResult()
	""" Result values: 0 - default/not started, 1-arrived, 2-arrived with trouble,
		3-aborted_home, 4-aborted_stuck, 5-preempted """
	

	# Add states to the master SM
	with sm:

		#Main move state to try to reach the goal		
		smach.StateMachine.add('USER_GOAL', UserGoalState(), 
				                   transitions={'outcome_movebase_arrived':'arrived',
												'outcome_movebase_arrived_trouble':'arrived_with_trouble', 
												'outcome_movebase_failed':'RECOVERY_SM',
												'outcome_movebase_failed_too_many':'GO_HOME'},
				                   remapping={'sPose':'kuser_goal',
											  'sRecCounter_in':'kRecCounter',
											  'sRecCounter_out':'kRecCounter',
											  'sPrevPose':'kPrevPose',
											  'sResult':'kuser_result'})

		#state to try and return home if recovery has failed
		smach.StateMachine.add('GO_HOME', mb_states.MoveBaseStateWithListandResult(frame = 'map'),
									transitions={'succeeded':'aborted_home',
												 'aborted':'aborted_stuck'},
									remapping={'xyt':'kHomePose',
											   'sResult':'kuser_result',
											   'sRecCounter_out':'kRecCounter'})
		

		#Create a sub-state machine for recovery		
		sm_recovery = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
										 input_keys=['kPrevPose'])

		#Initialize recovery userdata
		sm_recovery.userdata.kNearPose = NEAR_POSE

		with sm_recovery:
			
			#add state to set goal behind robot
			smach.StateMachine.add('GOAL_NEAR', mb_states.MoveBaseStateWithList(frame = 'base_link'),
										transitions={'succeeded':'succeeded',
													 'aborted':'REVERT_POSE'},
										remapping={'xyt':'kNearPose'})

			#add state to revert to previous pose
			smach.StateMachine.add('REVERT_POSE', mb_states.MoveBaseStateWithPose(), 
						               transitions={'succeeded':'succeeded',
													'aborted':'aborted'},
										remapping={'pose':'kPrevPose'})

		#set up recovery state machine to be connected just like a state
		smach.StateMachine.add('RECOVERY_SM',sm_recovery,
											transitions={'succeeded':'USER_GOAL',
														 'aborted':'GO_HOME'},
										 	remapping={'kPrevPose':'kPrevPose'})


	# Construct action server wrapper
	asw = smach_ros.ActionServerWrapper(
		'robust_controller_server', RobustControllerAction, sm,
		['arrived','arrived_with_trouble'], ['aborted_home','aborted_stuck'], ['preempted'],
		goal_key = 'kuser_goal' ,
		result_key = 'kuser_result')

	# Run the server in a background thread
	asw.run_server()

	# Wait for control-c
	rospy.spin()


if __name__ == '__main__':
	main()




