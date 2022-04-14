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
        self.rotations = {}

    def add_tag(self,id,x,y,z,theta_x,theta_y,theta_z):
        self.locations[id]=self.TranslationVector(x,y,z)
        self.orientations[id]=self.eulerAnglesToRotationMatrix(theta_x,theta_y,theta_z)
        self.rotations[id] = theta_y * (180/np.pi)

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
    
    def rotation_matrix_to_euler(self, R):
        y_rot = np.arcsin(R[2][0])
        x_rot = np.arccos(R[2][2]/np.cos(y_rot))
        z_rot = np.arccos(R[0][0]/np.cos(y_rot))
        y_rot_angle = y_rot * (180/np.pi) if not np.isnan(y_rot) else 0.0
        x_rot_angle = x_rot * (180/np.pi) if not np.isnan(x_rot) else 0.0
        z_rot_angle = z_rot * (180/np.pi) if not np.isnan(z_rot) else 0.0
        return np.array([[x_rot_angle], [y_rot_angle], [z_rot_angle]])

    def estimate_pose(self, tag_id: int, R: np.ndarray, t: np.ndarray):
        # Get Location And Orientation Of Tag
        tag_t = self.locations[tag_id]
        tag_R = self.orientations[tag_id]
        #rospy.loginfo(f"tag_R: {tag_R}")

        # Find Location
        location_in_tag_frame = R.transpose().dot(-1 * t)
        #rospy.loginfo(f"Location In Tag Frame: {location_in_tag_frame}")
        location_in_global_frame = tag_R.dot(location_in_tag_frame) + tag_t
        # rospy.loginfo(f"Location In Global Frame: {location_in_global_frame}")

        # Find Orientation
        tag_rotation = self.rotations[tag_id]
        rot = self.rotation_matrix_to_euler(R.transpose())
        rot[1,0] += tag_rotation
        rot[1,0] = ((rot[1,0] + 180) % 360) - 180

        return location_in_global_frame, rot
