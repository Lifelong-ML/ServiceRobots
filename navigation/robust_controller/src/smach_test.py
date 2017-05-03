#!/usr/bin/python

#==========================================================
# imports

import rospy
import smach, smach_ros
from geometry_msgs.msg import PoseStamped

"""
Naming conventions:
STATE_NAME
StateNameClass()
outcome_*
kStateMachineDataKey
sStateDataKey
"""



class MoveBaseState(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
                             outcomes=['outcome_movebase_arrived','outcome_movebase_failed'],
                             input_keys=['sPose'])

	def execute(self, userdata):
		rospy.loginfo('Executing state MOVE_BASE')
			
		##DO STUFF HERE
		status = True

		if status
			return 'outcome_movebase_arrived'
		else:
			return 'outcome_movebase_failed'


class GoHomeState(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
                             outcomes=['outcome_gohome_arrived','outcome_gohome_failed'])

	def execute(self, userdata):
		rospy.loginfo('Executing state GO_HOME')
		
		##DO STUFF HERE
		status = True

		if status
			return 'outcome_gohome_arrived'
		else:
			return 'outcome_gohome_failed'
	

class RecoveryState(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
                             outcomes=['outcome_recover_success','outcome_recover_failed'],
                             input_keys=['sPose','sPose_new'],
                             output_keys=['sPose_new'])

	def execute(self, userdata):
		rospy.loginfo('Executing state RECOVERY')
		if userdata.sPose.pose.position.x == 0:
			return 'outcome_recover_failed'
		else:
			userdata.sPose_new = userdata.sPose
			userdata.sPose_new.pose.position.x = 5 #note, to execute this line sPose_new must also be an input
			return 'outcome_recover_success'


def main():

	rospy.init_node('smach_example_state_machine')

	# Construct state machine
	sm = smach.StateMachine(outcomes=['arrived',
		                        'aborted',
		                        'preempted'])
	#initialize pose
	sm.userdata.kPose = PoseStamped()
	sm.userdata.kPose.pose.position.x = 0

	with sm:
		# Add states to the container

		smach.StateMachine.add('MOVE_BASE', MoveBaseState(), 
				                   transitions={'outcome_mb_arrived':'arrived', 
												'outcome_mb_failed':'RECOVERY_SM'},
				                   remapping={'sPose':'kPose'})
		
		sm_recovery = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
											input_keys=['kPose_recovery'])

		with sm_recovery:

			smach.StateMachine.add('RECOVERY', RecoveryState(), 
						               transitions={'outcome_recover_success':'succeeded',
													'outcome_recover_failed':'aborted'},
									   remapping={'sPose':'kPose_recovery',
												  'sPose_new':'kPose_recovery'})
	
		smach.StateMachine.add('RECOVERY_SM',sm_recovery,
											transitions={'succeeded':'MOVE_BASE',
															'aborted':'aborted'},
										 	remapping={'kPose_recovery':'kPose'})

	# Execute SMACH plan
	outcome = sm.execute()


if __name__ == '__main__':
	main()




