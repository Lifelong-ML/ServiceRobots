#!/usr/bin/env python

"""This file is modified from University of Applied Sciences Hamburg Robot Vision Lab ROS Repository
https://github.com/felix-kolbe/uashh-rvl-ros-pkg """

""" This file generates easy to use smach states needed to move the robot base. """

import rospy
import tf

import math
import random

import smach
from smach import State, Sequence
from smach_ros import SimpleActionState, ServiceState

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.srv import GetPlan, GetPlanRequest
from actionlib import GoalStatus

import util
from util import WaitForMsgState



def pose_orientation_to_quaternion(msg):
    """converts a geometry_msgs/Pose to a quaternion tuple/list
    e.g. to be used by tf.transformations.euler_from_quaternion
    """
    return [msg.x, msg.y, msg.z, msg.w]

def position_tuple_to_pose(x, y, yaw):
    """converts a position tuple to a geometry_msgs/Pose"""
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion(*quat)
    position = Point(x, y, 0)
    return Pose(position, orientation)


def calc_random_pose_tuple(distance_min=1, distance_max=3,
                           angular_deviation=(util.TAU / 2)):
    """
    angular_deviation: the direction space being straight ahead +-angular_deviation
    """
    distance = distance_min + (random.random() * (distance_max - distance_min))
    yaw = (random.random() * angular_deviation * 2) - (angular_deviation)
    x = math.cos(yaw) * distance
    y = math.sin(yaw) * distance
    return (x, y, yaw)



def get_move_base_in_map_state(x, y):
    return get_move_base_state("/map", x, y)

def get_move_base_in_odom_state(x, y):
    return get_move_base_state("/odom", x, y)

def get_move_base_random_state():
    """Note: each state returned is only randomized once at initialization and then static."""
    angular_deviation = util.TAU * 3 / 4 - util.TAU * 3 / 8    # +-180 deg
    x, y, yaw = calc_random_pose_tuple(angular_deviation=angular_deviation)
    return get_move_base_state("/base_link", x, y, yaw)


def get_move_base_state(frame='/map', x=0, y=0, yaw=0):
    """Return a MoveBaseGoal state which goal parameters are given via parameters at setup time."""
    print "new goal: ", x, y, yaw
    base_goal = MoveBaseGoal()
    base_goal.target_pose.header.frame_id = frame
    base_goal.target_pose.header.stamp = rospy.Time.now()

    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    base_goal.target_pose.pose.orientation = Quaternion(*quat)
    base_goal.target_pose.pose.position = Point(x, y, 0)

    return SimpleActionState('move_base',
                             MoveBaseAction,
                             goal=base_goal
                             )



class MoveBaseState(SimpleActionState):
    """Calls a move_base action server with the goal (x, y, yaw) from userdata"""
    def __init__(self, frame='/map'):
        SimpleActionState.__init__(self, 'move_base', MoveBaseAction, input_keys=['x', 'y', 'yaw'], goal_cb=self.__goal_cb)
        self.frame = frame

    def __goal_cb(self, userdata, old_goal):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame
        goal.target_pose.header.stamp = rospy.Time.now()

        quat = tf.transformations.quaternion_from_euler(0, 0, userdata.yaw)
        goal.target_pose.pose.orientation = Quaternion(*quat)
        goal.target_pose.pose.position = Point(userdata.x, userdata.y, 0)
        return goal

class MoveBaseStateWithList(SimpleActionState):
    """Calls a move_base action server with the goal as a list (or tuple) [x, y, yaw] from userdata"""
    def __init__(self, frame='/map'):
        SimpleActionState.__init__(self, 'move_base', MoveBaseAction, input_keys=['xyt'], goal_cb=self.__goal_cb)
        self.frame = frame

    def __goal_cb(self, userdata, old_goal):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame
        goal.target_pose.header.stamp = rospy.Time.now()

        quat = tf.transformations.quaternion_from_euler(0, 0, userdata.xyt[2])
        goal.target_pose.pose.orientation = Quaternion(*quat)
        goal.target_pose.pose.position = Point(userdata.xyt[0], userdata.xyt[1], 0)
        return goal


class MoveBaseStateWithListandResult(SimpleActionState):
    """Calls a move_base action server with the goal as a list (or tuple) [x, y, yaw] from userdata"""
    """This state has been modified to include a result callback specific to the robust controller node that uses it """
    def __init__(self, frame='/map'):
        SimpleActionState.__init__(self, 'move_base', MoveBaseAction, input_keys=['xyt','sResult'], goal_cb=self.__goal_cb,result_cb=self.movebase_result_cb,output_keys=['sResult','sRecCounter_out'])
        self.frame = frame

    def __goal_cb(self, userdata, old_goal):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame
        goal.target_pose.header.stamp = rospy.Time.now()

        quat = tf.transformations.quaternion_from_euler(0, 0, userdata.xyt[2])
        goal.target_pose.pose.orientation = Quaternion(*quat)
        goal.target_pose.pose.position = Point(userdata.xyt[0], userdata.xyt[1], 0)
        return goal
        
    def movebase_result_cb(self, userdata, status, result):
        if status == GoalStatus.SUCCEEDED:
            userdata.sResult.result = 3
            userdata.sRecCounter_out = 0
            return 'succeeded'
        else:
        	userdata.sResult.result = 4
        	userdata.sRecCounter_out = 0
        	return 'aborted'


class MoveBaseStateWithPose(SimpleActionState):
    """Calls a move_base action server with the goal as a PoseStamped() from userdata"""
    def __init__(self):
        SimpleActionState.__init__(self, 'move_base', MoveBaseAction, input_keys=['pose'], goal_cb=self.__goal_cb)

    def __goal_cb(self, userdata, old_goal):
        goal = MoveBaseGoal()
        goal.target_pose = userdata.pose
        return goal


class CheckForPlanState(ServiceState):
    """Check whether move_base can make a plan from start to goal given as
    tuples (x, y, yaw) via userdata"""
    def __init__(self, frame='/map'):
        ServiceState.__init__(self, 'move_base/make_plan', GetPlan,
                              input_keys=['x', 'y', 'yaw', 'start_x', 'start_y', 'start_yaw'],
                              request_cb=self.__request_cb)
        self.frame = frame

    def __request_cb(self, userdata, request):
        request = GetPlanRequest()
        request.goal.header.stamp = rospy.Time.now()
        request.goal.header.frame_id = self.frame
        request.goal.pose = position_tuple_to_pose(userdata.x, userdata.y, userdata.yaw)

        request.start.header.stamp = rospy.Time.now()
        request.start.header.frame_id = self.frame
        request.goal.pose = position_tuple_to_pose(userdata.start_x, userdata.start_y, userdata.start_yaw)
        #request.start.pose = position_tuple_to_pose(*util.get_current_robot_position(self.frame))

        request.tolerance = 0.2 # meters in x/y
        return request


class CalcRandomGoalState(State):
    """Return a random (x, y, yaw) tuple via userdata.

    (x,y) lies in the direction range of +-180 degrees.
    Radius range defaults to 1-3 m.
    """
    def __init__(self, radius_min=1, radius_max=3):
        State.__init__(self, outcomes=['succeeded'], output_keys=['x', 'y', 'yaw'])
        self.radius_min = radius_min
        self.radius_max = radius_max

    def execute(self, ud):
        ud.x, ud.y, ud.yaw = calc_random_pose_tuple(self.radius_min,
                                                    self.radius_max,
                                                    util.TAU / 2)  # 180 deg
        return 'succeeded'


def get_random_goal_smach(frame='/base_link'):
    """Return a SMACH Sequence for navigation to a newly randomly calulated goal.
    Combines CalcRandomGoalState with MoveBaseState

    frame: defaults to /base_link
    """
    sq = Sequence(outcomes=['succeeded', 'aborted', 'preempted'], connector_outcome='succeeded')

    sq.userdata.x = 0
    sq.userdata.y = 0
    sq.userdata.yaw = 0

    with sq:
        # implicit usage of above userdata
        Sequence.add("CALC_RANDOM_GOAL", CalcRandomGoalState())
        Sequence.add("MOVE_RANDOM_GOAL", MoveBaseState(frame))
    return sq




class WaitForGoalState(WaitForMsgState):
    def __init__(self):
        WaitForMsgState.__init__(self, '/move_base_task/goal', PoseStamped, self._msg_cb, output_keys=['x', 'y', 'yaw'])

    def _msg_cb(self, msg, ud):
        ud.x = msg.pose.position.x
        ud.y = msg.pose.position.y
        (_roll, _pitch, yaw) = tf.transformations.euler_from_quaternion(pose_orientation_to_quaternion(msg.pose.orientation))
        ud.yaw = yaw




class HasMovedState(State):
    """Return whether the robot moved beyond a given minimum distance in a given frame
    since the last exceeding check.

    minimum_distance: distance threshold to control outcomes
    frame: frame in which to retrieve the robot's pose, defaults to /map
    """
    def __init__(self, minimum_distance, frame='/map'):
        smach.State.__init__(self, outcomes=['movement_exceeds_distance', 'movement_within_distance'])
        util.TransformListenerSingleton.init()
        self.minimum_distance = minimum_distance
        self.frame = frame
        self.lastX, self.lastY = self._getXY()

    def _getXY(self):
        x, y, _yaw = util.get_current_robot_position(self.frame)
        return x, y

    def execute(self, userdata):
        currentX, currentY = self._getXY()
        current_distance = math.sqrt(math.pow(currentX, 2) + math.pow(currentY, 2))
        rospy.logdebug("current XY: %f,%f last XY: %f,%f current distance: %f minimum distance: %f",
                       self.lastX, self.lastY, currentX, currentY, current_distance, self.minimum_distance)
        if current_distance >= self.minimum_distance:
            self.lastX = currentX
            self.lastY = currentY
            return 'movement_exceeds_distance'
        else:
            return 'movement_within_distance'


class ReadRobotPositionState(State):
    """Return the current robot position in the given frame via userdata.

    frame: defaults to /map
    """
    def __init__(self, frame='/map'):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['x', 'y', 'yaw'])
        self.frame = frame
        util.TransformListenerSingleton.init()

    def execute(self, userdata):
        userdata.x, userdata.y, userdata.yaw = util.get_current_robot_position(self.frame)
        return 'succeeded'
