#!/usr/bin/env python

"""This file is modified from University of Applied Sciences Hamburg Robot Vision Lab ROS Repository
https://github.com/felix-kolbe/uashh-rvl-ros-pkg """


""" This file contains general purpose utility states and methods. """

import rospy
import tf
from std_msgs.msg import Bool

import math
import threading

import smach
import smach_ros

import rostopic



TAU = math.pi * 2   # one tau is one turn. simply as that.




class PromptState(smach.State):
    """Prompt and wait for user action or input on command line.

    prompt: String to display. If not given or None, prompt is
                    read from userdata key 'duration'.

    userdata input prompt: message displayed at prompt (not registered
                    as input key if prompt given on initialization)
    userdata output user_input: where the user input is returned
    """
    def __init__(self, prompt=None):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['prompt'] if prompt is None else [],
                             output_keys=['user_input'])
        self.prompt = prompt

    def execute(self, userdata):
        rospy.loginfo('Executing PromptState')
        prompt = userdata.prompt if self.prompt is None else self.prompt
        try:
            userdata.user_input = raw_input(prompt)
            return 'succeeded'
        except EOFError:
            return 'aborted'


class SleepState(smach.State):
    """Sleep for a time duration, given either on initialization or via userdata.

    duration: of type rospy Duration or float in seconds. If not given or None,
                duration is read from userdata key 'duration'.

    userdata input duration: of type rospy Duration or float in seconds (not
                registered as input key if given on initialization)
    """
    def __init__(self, duration=None):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
                             input_keys=['duration'] if duration is None else [])
        self.duration = duration

    def execute(self, userdata):
        duration = userdata.duration if self.duration is None else self.duration
        rospy.loginfo("SleepState sleeping for %d seconds" % duration)
        # sleep in steps to handle state preemption
        SLEEP_STEP = 2 # maximum to sleep per step
        while duration > 0:
            sleeptime = SLEEP_STEP if duration > SLEEP_STEP else duration
            duration -= sleeptime
            rospy.sleep(sleeptime)
            if self.preempt_requested():
                self.service_preempt()
                rospy.loginfo('SleepState was preempted while sleeping!')
                return 'preempted'
        return 'succeeded'



class WaitForMsgState(smach.State):
    """This class acts as a generic message listener with blocking, timeout, latch and flexible usage.

    It is meant to be extended with a case specific class that initializes this one appropriately
    and contains the msg_cb (or overrides execute if really needed).

    Its waitForMsg method implements the core functionality: waiting for the message, returning
    the message itself or None on timeout.

    Its execute method wraps the waitForMsg and returns succeeded or aborted, depending on the returned
    message beeing existent or None. Additionally, in the successfull case, the msg_cb, if given, will
    be called with the message and the userdata, so that a self defined method can convert message data to
    smach userdata.
    Those userdata fields have to be passed via 'output_keys'.

    If the state outcome should depend on the message content, the msg_cb can dictate the outcome:
    If msg_cb returns True, execute() will return "succeeded".
    If msg_cb returns False, execute() will return "aborted".
    If msg_cb has no return statement, execute() will act as described above.

    If thats still not enough, execute() might be overridden.

    latch: If True waitForMsg will return the last received message, so one message might be returned indefinite times.
    timeout: Seconds to wait for a message, defaults to None, disabling timeout
    output_keys: Userdata keys that the message callback needs to write to.
    """

    def __init__(self, topic, msg_type, msg_cb=None, output_keys=None, latch=False, timeout=None):
        if output_keys is None:
            output_keys = []
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], output_keys=output_keys)
        self.latch = latch
        self.timeout = timeout
        self.mutex = threading.Lock()
        self.msg = None
        self.msg_cb = msg_cb
        self.subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size=1)

    def _callback(self, msg):
        self.mutex.acquire()
        self.msg = msg
        self.mutex.release()

    def waitForMsg(self):
        """Await and return the message or None on timeout."""
        rospy.loginfo('Waiting for message...')
        if self.timeout is not None:
            timeout_time = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
        while self.timeout is None or rospy.Time.now() < timeout_time:
            self.mutex.acquire()
            if self.msg is not None:
                rospy.loginfo('Got message.')
                message = self.msg

                if not self.latch:
                    self.msg = None

                self.mutex.release()
                return message
            self.mutex.release()

            if self.preempt_requested():
                self.service_preempt()
                rospy.loginfo('waitForMsg is preempted!')
                return 'preempted'

            rospy.sleep(.1) # TODO: maybe convert ROSInterruptException into valid outcome

        rospy.loginfo('Timeout on waiting for message!')
        return None

    def execute(self, ud):
        """Default simplest execute(), see class description."""
        msg = self.waitForMsg()
        if msg is not None:
            if msg == 'preempted':
                return 'preempted'
            # call callback if there is one
            if self.msg_cb is not None:
                cb_result = self.msg_cb(msg, ud)
                # check if callback wants to dictate output
                if cb_result is not None:
                    if cb_result:
                        return 'succeeded'
                    else:
                        return 'aborted'
            return 'succeeded'
        else:
            return 'aborted'



class CheckSmachEnabledState(WaitForMsgState):
    def __init__(self, **kwargs):
        WaitForMsgState.__init__(self, '/enable_smach', Bool, msg_cb=self._msg_cb, latch=True, **kwargs)

    def _msg_cb(self, msg, ud):
        return msg is not None and msg.data


def get_sleep_until_smach_enabled_smach():
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted'])
    with sm:
        smach.StateMachine.add('CHECK_ENABLED',
                               CheckSmachEnabledState(timeout=None),
                               transitions={'aborted':'SLEEP',
                                            'succeeded':'succeeded'})
        smach.StateMachine.add('SLEEP',
                               SleepState(3),
                               transitions={'succeeded':'CHECK_ENABLED'})
    return sm



class TopicToOutcomeState(smach.State):
    """This state returns the message value for given topic/field received from
    the wrapped WaitForMsgState as its outcome.
    """
    def __init__(self, outcomes, topic, topic_class, field, **kwargs):
        outcomes = outcomes[:] # copy needed to extend list
        outcomes.extend(['timeout', 'field_error', 'undefined_outcome'])
        smach.State.__init__(self, outcomes)
        self._wait_for_msg_state = WaitForMsgState(topic, topic_class, output_keys=outcomes, **kwargs)
        self._outcomes = outcomes
        self._msgeval = rostopic.msgevalgen(field)

    def execute(self, ud):
        msg = self._wait_for_msg_state.waitForMsg()
        if msg is not None:
            field = self._msgeval(msg)
            print "got field as: ", field
            if field is None:
                return 'field_error'
            elif field in self._outcomes:
                return field
            else:
                return 'undefined_outcome'
        else:
            return 'timeout'



class UserDataToOutcomeState(smach.State):
    """This state returns the userdata value for given input_key as its outcome.
    """
    def __init__(self, outcomes, input_key, ud_to_value_func):
        outcomes = outcomes[:] # copy needed to extend list
        outcomes.extend(['field_error', 'undefined_outcome'])
        smach.State.__init__(self, outcomes, [input_key])
        self._outcomes = outcomes
        self._input_key = input_key
#        self._field = 'task_id'
        self._ud_to_value_func = ud_to_value_func

    def execute(self, ud):
        field = self._ud_to_value_func(ud[self._input_key])
#        field = ud[self._input_key][self._field]
        print "got field as: ", field
        if field is None:
            return 'field_error'
        elif field in self._outcomes:
            return field
        else:
            return 'undefined_outcome'



class TransformListenerSingleton(object):
    """To avoid running multiple transform listeners, this singleton class
    provides one transform listener that is initialised and retrieved via
    class methods init() and get().
    """
    _transform_listener = None

    @classmethod
    def init(cls):
        """Ignores multiple calls."""
        if cls._transform_listener is None:
            cls._transform_listener = tf.TransformListener()

    @classmethod
    def get(cls):
        """Does initialise if needed, too."""
        cls.init()
        return cls._transform_listener



def get_current_robot_position_in_odom_frame():
    return get_current_robot_position('/odom')

def get_current_robot_position(frame='/map'):
    """Returns a (x,y,yaw) tuple for the robot in a given frame.
    frame: defaults to /map
    """
    try:
        trans, rot = TransformListenerSingleton.get().lookupTransform(frame, '/base_link', rospy.Time(0))
        (_roll, _pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
        return trans[0], trans[1], yaw
    except (tf.LookupException, tf.ConnectivityException) as e:
        print e
        return 0, 0, 0
    # TODO: i.e. forward exception



def execute_smach_container(smach_container, enable_introspection=False,
                            name='/SM_ROOT', userdata=smach.UserData()):
    if not rospy.core.is_initialized():
        rospy.init_node('smach')

    if enable_introspection:
        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('smach_executor', smach_container, name)
        sis.start()

    outcome = smach_container.execute(userdata)
    print 'smach outcome: ', outcome

    if enable_introspection:
        # Wait for ctrl-c to stop the application
        rospy.spin() # TODO: remove spinning or make optional
        sis.stop()

    return outcome

def simple_state_wrapper(state):
    """Wrap a minimal state machine around the given state to make it
    executable with all SMACH features"""
    sm = smach.StateMachine(list(state.get_registered_outcomes()),
                            list(state.get_registered_input_keys()),
                            list(state.get_registered_output_keys()))
    with sm:
        smach.StateMachine.add('SIMPLE_STATE_WRAPPED', state)
    return sm
