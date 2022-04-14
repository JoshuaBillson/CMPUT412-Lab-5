#!/usr/bin/env python3

import time
import os
import math
import rospy
from std_msgs.msg import Int32
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion, Point, Pose, Vector3
from tf.transformations import euler_from_quaternion
from threading import Lock

POSE_TOPIC = "csc22912/output/position"
ORIENTATION_TOPIC = "csc22912/output/orientation"
TAG_ID_TOPIC = "csc22912/output/tag_id"

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pose_subscriber = rospy.Subscriber(POSE_TOPIC, Vector3, self.pose_callback, queue_size=1)
        self.orientation_subscriber = rospy.Subscriber(ORIENTATION_TOPIC, Vector3, self.orientation_callback, queue_size=1)
        self.tag_subscriber = rospy.Subscriber(TAG_ID_TOPIC, Int32, self.tag_callback, queue_size=1)
        self.mutex = Lock()
        self.tag_id = None
        self.position = (0, 0, 0)
        self.orientation = (0, 0, 0)
    
    def pose_callback(self, data: Vector3):
        with self.mutex:
            self.position = (data.x, data.y, data.z)

    def orientation_callback(self, data: Vector3):
        with self.mutex:
            self.orientation = (data.x, data.y, data.z)
    
    def tag_callback(self, data: Int32):
        self.tag_id = data.data

    def run(self):
        count = 0
        rate = rospy.Rate(10)
        locations = [(0, 0, 0) for i in range(10)]
        orientations = [(0, 0, 0) for i in range(10)]
        while not rospy.is_shutdown():
            with self.mutex:
                locations.pop(0)
                locations.append(self.position)
                orientations.pop(0)
                orientations.append(self.orientation)
            
            count += 1
            if count == 10:
                location = (sum([x[0] for x in locations]), sum([x[1] for x in locations]), sum([x[2] for x in locations]))
                orientation = (sum([x[0] for x in orientations]), sum([x[1] for x in orientations]), sum([x[2] for x in orientations]))
                rospy.loginfo(f"Location: {tuple(map(lambda x: round(x / count, 2), location))}   Orientation: {tuple(map(lambda x: round(x / count, 2), orientation))}")
                count = 0
            rate.sleep()
    

if __name__ == '__main__':
    node = MyPublisherNode(node_name='lab_5_main_node')
    node.run()
