#!/usr/bin/python

#==========================================================
# imports

import rospy
from map_labelling.srv import GoToLocation, GoToLocationResponse, MotionStatus,MotionStatusResponse
import random


locations = [ 	"grasp",
				"charitys_office",
				"vending_machines",
				"levine_far_corner",
				"towne_311",
				"towne_321",
				"to_skirkanich",
				"empty_spot",
				"bio_lab" ]
					


#====================================================
# node exectution

def runNode():

    #start node, initial localization is hard coded into launch file
    rospy.init_node('torture_test')
    rospy.loginfo("torture_test node initialized")

    rospy.wait_for_service('goto_location')
    rospy.wait_for_service('motion_status')
    rospy.loginfo("Navigation services are available")  

    LocationList = [Location(s) for s in locations] 

    rospy.sleep(10)
    rospy.loginfo("Beginning tests")

    #keep running while alive
    while not rospy.is_shutdown():
		
		#pick random place to go
		L = random.choice(LocationList)
		L.goto()
		L.print_status()
		

#======================================
# Location class

#Information and methods pertaining to a location
class Location:

	def __init__(self,name):
		self.name = name
		self.goToService = rospy.ServiceProxy('goto_location', GoToLocation)
		self.motionStatus = rospy.ServiceProxy('motion_status',MotionStatus)
		self.status = False

	#Navigate to this location
	def goto(self):
	    rospy.loginfo("Navigating to "+self.name)
	    
	    try:
			self.goToService(self.name)
	    except rospy.service.ServiceException:
			rospy.logerr("Location " + self.name+ " does not exist.")
                        return

	    #wait for arrival
	    done = False
	    while not done:
		
			rospy.sleep(1)
			motionStatus = self.motionStatus().status

			#0=PENDING,1=ACTIVE so these are considered running, anything else means the goal is no longer running
			if not(motionStatus == 0 or motionStatus == 1):
				done = True
				if motionStatus == 3:  #3 is succeeded motion
					self.status = True
				else:
					self.status = False

	def print_status(self):
		if self.status:
			rospy.loginfo("Navigating to "+self.name+" succeeded")
		else:
			rospy.loginfo("Navigating to "+self.name+" failed")


#=================================
# Run main

if __name__ == '__main__':
    try:
        runNode()
    except rospy.ROSInterruptException:
         pass
