#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from map_labelling.srv import LabelMap, LabelMapResponse, NearestLocation, NearestLocationResponse
from map_labelling.srv import GoToLocation, GoToLocationResponse, DistanceToLocation, DistanceToLocationResponse, MotionStatus, MotionStatusResponse
from robust_controller.msg import RobustControllerGoal, RobustControllerAction, RobustControllerResult
from std_msgs.msg import String
import json
import actionlib
import math
import Levenshtein

current_pose = None
saved_locations = {}
client = None #global move base client for motion checking

def label_map(msg):
	global label_publisher, saved_locations, current_pose
	label = msg.label 
	if current_pose:
		print "label: %s, x: %f, y: %f" % (label, current_pose.position.x, current_pose.position.y)
		saved_locations[label] =  {'x': current_pose.position.x, 'y': current_pose.position.y,
									'orientation': (current_pose.orientation.x,
													current_pose.orientation.y,
													current_pose.orientation.z,
													current_pose.orientation.w)}
		return {'x': current_pose.position.x, 'y': current_pose.position.y}
	else:
		print "no pose saved yet"
		return {'x': 0, 'y': 0}
	save_locations_file()
	label_publisher.publish(String(','.join(saved_locations.keys())))
	print ','.join(saved_locations.keys())	

def get_current_location(msg):
	global current_pose
	current_pose = msg.pose.pose

def get_nearest_location(msg):
	global current_pose, saved_locations 
	# do this in a sloppy way
	min_dist = 1000
	cur_x = current_pose.position.x 
	cur_y = current_pose.position.y
	x = 0 
	y = 0
	closest_label = ""
	for (label, location) in saved_locations.iteritems():
		dist = (cur_x - float(location['x']))**2 + (cur_y - float(location['y']))**2 
		if dist < min_dist:
			min_dist = dist
			x = abs(cur_x - location['x'])
			y = abs(cur_y - location['y'])
			closest_label = label 
	return {'label': closest_label, 'xDistance': x, 'yDistance': y}

def go_to(msg):
	global saved_locations, client
	label = msg.label
	levenshtein_threshold = 0.5
	#lev_distances = [Levenshtein.ratio(label, loc) for loc in saved_locations.keys()]
	#lev_distance_best = max(lev_distances)
	
	#if lev_distance_best >= levenshtein_threshold:
	#	label = saved_locations[lev_distances.index(lev_distance_best)]
	#else:
	#	return None	

	if label not in saved_locations:
	    return None
	client = actionlib.SimpleActionClient('robust_controller_server',RobustControllerAction)
	client.wait_for_server()

	actionGoal = RobustControllerGoal()
	actionGoal.target_pose.header.frame_id = "map"
	actionGoal.target_pose.header.stamp = rospy.get_rostime()

	actionGoal.target_pose.pose.position.x = saved_locations[label]['x']#msg.pose.pose.position.x
	actionGoal.target_pose.pose.position.y = saved_locations[label]['y']#msg.pose.pose.position.y
	actionGoal.target_pose.pose.orientation.x = saved_locations[label]['orientation'][0]
	actionGoal.target_pose.pose.orientation.y = saved_locations[label]['orientation'][1]
	actionGoal.target_pose.pose.orientation.z = saved_locations[label]['orientation'][2]
	actionGoal.target_pose.pose.orientation.w = saved_locations[label]['orientation'][3]

	client.send_goal(actionGoal)
	# client.wait_for_result()
	return {'xGoal': saved_locations[label]['x'], 'yGoal': saved_locations[label]['y']}

def stop():
	client = actionlib.SimpleActionClient('robust_controller_server',RobustControllerAction)
	client.wait_for_server()

	client.cancel_all_goals()

def checkMotion(msg):
	global client
	return client.get_state()
	

def distance_to(msg):
	global saved_locations
	cur_x = current_pose.position.x
	cur_y = current_pose.position.y
	location = saved_locations[msg.label]
	x = abs(cur_x - location['x'])
	y = abs(cur_y - location['y'])
	distance = math.sqrt(x ** 2 + y ** 2)
	return {'dist': distance}

def load_saved_locations():
	global saved_locations, label_file
	with open(label_file, 'r') as f:
		saved_locations = json.load(f)
	print "saved_locations" + str(saved_locations)

def save_locations_file():
	global saved_locations, label_file
	print saved_locations
	with open(label_file, 'w') as f:
		f.write(json.dumps(saved_locations))

#==============================
#run main section
rospy.init_node('map_labeller')
#check if label file parameter exists
if rospy.has_param('~label_file'):

	#load label file
	label_file = rospy.get_param('~label_file')
	try: 
		load_saved_locations()
	except: 
		print 'no saved locations!'
else:
	print 'no label file parameter found'
	label_file = 'default_label.json'
	

#pose subsrciption
sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, get_current_location)

#advertise services
label_service = rospy.Service('label_map', LabelMap, label_map)
nearest_location_service = rospy.Service('get_nearest_location', NearestLocation, get_nearest_location)
goto_service = rospy.Service('goto_location', GoToLocation, go_to)
status_service = rospy.Service('motion_status',MotionStatus,checkMotion)
distance_to_service = rospy.Service('distance_to_location', DistanceToLocation, distance_to)

#publish labeled locations
label_publisher = rospy.Publisher('labelled_locations', String, queue_size=10)
label_publisher.publish(String(','.join(saved_locations.keys())))
print ','.join(saved_locations.keys())

#wait and chill
rospy.spin()

#save file when done
save_locations_file()
