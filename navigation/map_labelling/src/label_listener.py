#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance

def callback(msg):
	print "x: " + str(msg.pose.pose.position.x)
	print "y: " + str(msg.pose.pose.position.y)

rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, callback)
rospy.spin()
