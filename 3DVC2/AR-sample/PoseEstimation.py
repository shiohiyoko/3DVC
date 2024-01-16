import numpy as np
import cv2

class PoseEstimation:
    def __init__(self, f, u0, v0):
        self.A = np.array([[f, 0.0, u0], [0.0, f, v0], [0.0, 0.0, 1.0]], dtype = "double")
        self.dist_coeff = np.zeros((4, 1))
        
    def compute_camera_pose(self):
        # not implemented
        # Please implement yourself
        return False, None, None
