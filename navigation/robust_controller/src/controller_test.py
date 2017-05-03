#!/usr/bin/python

import rospy
import actionlib
from robust_controller.msg import RobustControllerAction, RobustControllerGoal
import math

Locations = [(0,0,0),(1,0,0),(2,-1,1)]


#Set up a quick test
def runDemo():

    rospy.init_node('controller_test')
    rospy.loginfo("Node initialized")    

    for loc in Locations:
		direct_move(loc[0],loc[1],loc[2],"map")

    rospy.loginfo("Done")
    rospy.spin()



#Send direct motion command to the xya coordinates in the current frame
def direct_move(x,y,yaw,frame):
	client = actionlib.SimpleActionClient('robust_controller_server', RobustControllerAction)
	client.wait_for_server() 

	actionGoal = RobustControllerGoal()
	actionGoal.target_pose.header.frame_id = frame
	actionGoal.target_pose.header.stamp = rospy.get_rostime()

	actionGoal.target_pose.pose.position.x = x 
	actionGoal.target_pose.pose.position.y = y 
        actionGoal.target_pose.pose.orientation.z = math.sin(yaw/2)
        actionGoal.target_pose.pose.orientation.w = math.cos(yaw/2)
	print actionGoal
	client.send_goal(actionGoal)
	client.wait_for_result()
	rslt = client.get_result()
	print "Robust Controller Result is: "+str(rslt.result)


if __name__ == '__main__':
    try:
        runDemo()
    except rospy.ROSInterruptException:
         pass
