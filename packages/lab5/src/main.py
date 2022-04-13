#!/usr/bin/env python3

import time
import os
import rospy
from std_msgs.msg import Float32, Int32, String
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
from std_msgs.msg import String
import smach
from threading import Lock, Thread

MOTOR_TOPIC = "/csc22912/car_cmd_switch_node/cmd"
LEFT_MOTOR_TOPIC = "/csc22912/wheels_driver_node/wheels_cmd"
LINE_TRACKER_TOPIC = "/csc22912/output/line_tracker"
STOP_MATCHER_TOPIC = "/csc22912/output/stop_matcher"
TOPIC = "csc22912/output/msg"
VELOCITY = 0.30

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher(TOPIC, String, queue_size=10)

    def run(self):

        counter = 0
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            msg = String()
            msg.data = f"Hello World {counter}!"
            counter += 1
            self.pub.publish(msg)
            rate.sleep()
    
        # Open the container
        """
        with sm:
            # Add states to the container
            smach.StateMachine.add('FOLLOW_PATH', FollowPath(motor_controller, line_tracker, stop_matcher), transitions={'stop':'STOP'})
            smach.StateMachine.add('STOP', Stop(motor_controller, line_tracker, stop_matcher), transitions={'finish':'FINISH'})
    
        # Execute SMACH plan
        outcome = sm.execute()
        """


if __name__ == '__main__':
    node = MyPublisherNode(node_name='lab_5_main_node')
    node.run()
