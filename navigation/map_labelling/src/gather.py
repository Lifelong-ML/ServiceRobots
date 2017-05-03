#!/usr/bin/python

import sys
import Queue
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from map_labelling.srv import LabelMap, LabelMapResponse, NearestLocation, NearestLocationResponse
from map_labelling.srv import GoToLocation, GoToLocationResponse, DistanceToLocation, DistanceToLocationResponse
from map_labelling.srv import MotionStatus, MotionStatusResponse
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseResult
import json
import actionlib
import time
import math

if __name__ == "__main__":
    goals = ["table1", "table2", "table1", "table3", "table1", "table4", "table1"]
    epsilon = 1.0
    print "waiting"
    rospy.wait_for_service('motion_status')
    rospy.wait_for_service('distance_to_location')
    print "done waiting"
    for i in range(0, len(goals)):
        try:
            goto_service = rospy.ServiceProxy('goto_location', GoToLocation)
            resp1 = goto_service(goals[i])

            status_service = rospy.ServiceProxy('motion_status', MotionStatus)

            resp2 = status_service()
            status = resp2.status
            print status
            while (status == 1):
                time.sleep(1)
                resp2 = status_service()
                status = resp2.status
                print status

            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
