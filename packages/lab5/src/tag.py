from math import sin, cos
import numpy as np
import rospy
import rospkg

''' Adapted from: https://raceon.io/localization/ '''

class Tag():
    def __init__(self, tag_size, family):
        self.family = family
        self.size = tag_size
        self.locations = {}
        self.orientations = {}
    

    def add_tag(self,id,x,y,z,theta_x,theta_y,theta_z):
        self.locations[id]=self.TranslationVector(x,y,z)
        self.orientations[id]=self.eulerAnglesToRotationMatrix(theta_x,theta_y,theta_z)

        
    # Calculates Rotation Matrix given euler angles.
    def eulerAnglesToRotationMatrix(self, theta_x, theta_y, theta_z):
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(theta_x), -np.sin(theta_x)],
                        [0, np.sin(theta_x), np.cos(theta_x)]
                        ])

        R_y = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                        [0, 1, 0],
                        [-np.sin(theta_y), 0, np.cos(theta_y)]
                        ])

        R_z = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                        [np.sin(theta_z), np.cos(theta_z), 0],
                        [0, 0, 1]
                        ])

        R = np.matmul(R_z, np.matmul(R_y, R_x))

        return R.T

    def TranslationVector(self,x,y,z):
        return np.array([[x],[y],[z]])

    def estimate_pose(self, tag_id: int, R: np.ndarray, t: np.ndarray):
        # Get Location And Orientation Of Tag
        x, y, z = self.locations[tag_id]
        tag_t = np.array([x, y, z])
        _, theta, _ = self.orientations[tag_id]

        # Create Rotation Matrix For Transforming From The Tag Frame To The Global Frame
        tag_R = np.array([[cos(theta), 0, -sin(theta)], [0, 1, 0], [sin(theta), 0, cos(theta)]])

        # Find Location
        location_in_tag_frame = R.transpose.dot(-1 * t)
        location_in_global_frame = tag_R.dot(location_in_tag_frame) - tag_t
        
        # Log For Debugging
        rospy.loginfo(f"Tag ID: {tag_id}")
        rospy.loginfo(f"R: {R}")
        rospy.loginfo(f"t: {t}")

        return location_in_global_frame
