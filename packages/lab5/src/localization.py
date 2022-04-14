#!/usr/bin/env python3

import os
from typing_extensions import Self
import numpy as np
import yaml
import cv2
import math
import os
import rospy
from threading import Thread, Lock
from tag import Tag
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Quaternion, Point, Pose, Vector3
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from dt_apriltags import Detector
from duckietown.dtros import DTROS, NodeType

POSE_TOPIC = "csc22912/output/position"
ORIENTATION_TOPIC = "csc22912/output/orientation"
TAG_ID_TOPIC = "csc22912/output/tag_id"


class LocalizationNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LocalizationNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # April Tag Thread
        self.thread = Thread(target=self.detect_loop)
        self.mutex = Lock()
        self.img = None
        self.location = np.array([[0], [0], [0]])
        self.rotation = np.array([[0], [0], [0]])

        # Initialize Detector
        self.at_detector = Detector(searchpath=['apriltags'],
                                    families='tagStandard41h12',
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
     
        # Add your subsribers or publishers here
        self.subscriber = rospy.Subscriber("/csc22912/camera_node/image/compressed", CompressedImage, self.imgCallback, queue_size=1)
        self.location_publisher = rospy.Publisher(POSE_TOPIC, Vector3, queue_size=10)
        self.orientation_publisher = rospy.Publisher(ORIENTATION_TOPIC, Vector3, queue_size=10)
        self.id_publisher = rospy.Publisher(TAG_ID_TOPIC, Int32, queue_size=10)

        # Add information about tags
        TAG_SIZE = .08
        FAMILIES = "tagStandard41h12"
        self.tags = Tag(TAG_SIZE, FAMILIES)
        self.tags.add_tag(0, 0, 0, 0.3048, 0, math.pi / 2, 0)
        self.tags.add_tag(1, 0.3048, 0, 0.6096, 0, 0, 0)
        self.tags.add_tag(2, 0.6096, 0, 0.3048, 0, (3 * math.pi / 2), 0)
        self.tags.add_tag(3, 0.3048, 0, 0, 0, math.pi - 0.1, 0)

        # Load camera parameters
        with open("/data/config/calibrations/camera_intrinsic/csc22912.yaml") as file:
                camera_list = yaml.load(file,Loader = yaml.FullLoader)

        self.camera_intrinsic_matrix = np.array(camera_list['camera_matrix']['data']).reshape(3,3)
        self.distortion_coeff = np.array(camera_list['distortion_coefficients']['data']).reshape(5,1)
    
    def imgCallback(self, ros_data):
        '''
        This Callback Runs Whenever A New Image Is Published From The Camera.
        We Use This To Detect The Centroid Of The Path As Well As Its Colour.
        '''
        #### direct conversion to CV2 ####
        with self.mutex:
            np_arr = np.fromstring(ros_data.data, np.uint8)
            self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

    def detect_loop(self):
        """Loop To Detect Location From Current Camera Image"""
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():

            # Copy The Camera Image
            if self.img is not None:
                with self.mutex:
                    img = np.copy(self.img)
            else:
                img = None
        
            # Process Image
            if img is not None:

                # Prepare Image For Processing
                undistorted_image = self.undistort(img)
                grayscale_image = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)

                # Detect Tags
                count = 0
                tags = self.detect(grayscale_image)
                location = np.array([[0.0], [0.0], [0.0]])
                rotation = np.array([[0.0], [0.0], [0.0]])

                # For Each Detected Tag, Find The Global Coordinates And Take The Average
                for tag in tags:
                    l, r = self.tags.estimate_pose(tag.tag_id, tag.pose_R, tag.pose_t)
                    location += l
                    rotation += r
                    count += 1

                # Publish Tag ID
                if len(tags) > 0:
                    tag = tags[0]
                    tag_id_msg = Int32(tag.tag_id)
                    self.id_publisher.publish(tag_id_msg)

                # If No Tags Were Detected, The Location Does Not Change
                self.location = location / count if count > 0 else self.location
                self.rotation = rotation / count if count > 0 else self.rotation

            # Publish Pose And Tag
            position_msg = Vector3()
            position_msg.x = self.location[0,0]
            position_msg.y = self.location[1,0]
            position_msg.z = self.location[2,0]
            self.location_publisher.publish(position_msg)

            # Publish Orientation
            orientation_msg = Vector3()
            orientation_msg.x = self.rotation[0,0]
            orientation_msg.y = self.rotation[1,0]
            orientation_msg.z = self.rotation[2,0]
            self.orientation_publisher.publish(orientation_msg)

            #rospy.loginfo(f"Rotation: ({self.rotation[0,0]}, {self.rotation[1,0]}, {self.rotation[2,0]})")
            #rospy.loginfo(f"Location: ({self.location[0,0]}, {self.location[1,0]}, {self.location[2,0]})")
            rate.sleep()

    def run(self):
        self.thread.start()
        rospy.spin()
        self.thread.join()

    def undistort(self, img):
        '''
        Takes a fisheye-distorted image and undistorts it

        Adapted from: https://github.com/asvath/SLAMDuck
        '''
        height = img.shape[0]
        width = img.shape[1]

        newmatrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_intrinsic_matrix,
            self.distortion_coeff, 
            (width, height),
            1, 
            (width, height))

        map_x, map_y = cv2.initUndistortRectifyMap(
            self.camera_intrinsic_matrix, 
            self.distortion_coeff,  
            np.eye(3), 
            newmatrix, 
            (width, height), 
            cv2.CV_16SC2)

        undistorted_image = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)
       
        return undistorted_image   
             

    def detect(self, img):
        ''' Takes an images and detects AprilTags '''
        PARAMS = [
            self.camera_intrinsic_matrix[0,0],
            self.camera_intrinsic_matrix[1,1],
            self.camera_intrinsic_matrix[0,2],
            self.camera_intrinsic_matrix[1,2]] 
        
        return self.at_detector.detect(img, estimate_tag_pose=True, camera_params=PARAMS, tag_size=0.08)


def main():
    node = LocalizationNode(node_name='lab_5_localization_node')
    node.run()


if __name__ == "__main__":
    main()
