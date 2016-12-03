#!/usr/bin/python
import cv2
import numpy as np
import rospkg
import os

from yaml import load, Loader

def to_matrix(data):
    rows = data['rows']
    cols = data['cols']
    return np.reshape(data['data'], (rows,cols)).astype(np.float32)

def get_matrices(file_path):
    with open(file_path) as stream:
        data = load(stream, Loader=Loader)
        m = to_matrix(data['camera_matrix'])
        d = to_matrix(data['distortion_coefficients'])
        r = to_matrix(data['rectification_matrix'])
        p = to_matrix(data['projection_matrix'])
        return m, d, r, p

class Rectifier(object):
    def __init__(self, param_l, param_r):
        m_l, d_l, r_l, p_l = get_matrices(param_l)
        m_r, d_r, r_r, p_r = get_matrices(param_r)
        self.c_l_m1, self.c_l_m2 = cv2.initUndistortRectifyMap(m_l,d_l,r_l,p_l, (640,480), cv2.CV_32FC1)
        self.c_r_m1, self.c_r_m2 = cv2.initUndistortRectifyMap(m_r,d_r,r_r,p_r, (640,480), cv2.CV_32FC1)
    def apply(self, left, right):
        im_l = cv2.remap(left, self.c_l_m1, self.c_l_m2, cv2.INTER_LINEAR)
        im_r = cv2.remap(right, self.c_r_m1, self.c_r_m2, cv2.INTER_LINEAR)
        return im_l, im_r

class SGBM(object):
    def __init__(self):
        # fixed parameters for current oCam setup
        # StereoSGBM([minDisparity, numDisparities, SADWindowSize[, P1[, P2[, disp12MaxDiff[, preFilterCap[, uniquenessRatio[, speckleWindowSize[, speckleRange[, fullDP]]]]]]]]]) -> <StereoSGBM object>
        self.sgbm = cv2.StereoSGBM(0, 128, 9, 4000, 4000, 0, 31, 4, 1000, 5, False)
    def apply(self, im_l, im_r):
        disp = self.sgbm.compute(im_l, im_r)
        return cv2.normalize(disp,None,0,255,cv2.NORM_MINMAX).astype(np.uint8)

if __name__ == "__main__":
    # Usage example
    rospack = rospkg.RosPack()
    pkg_root = rospack.get_path('edwin')

    cam_l = cv2.VideoCapture(1)
    cam_r = cv2.VideoCapture(2)

    rect = Rectifier(
            param_l = os.path.join(pkg_root, 'Stereo/camera_info/left_camera.yaml'),
            param_r = os.path.join(pkg_root, 'Stereo/camera_info/right_camera.yaml')
            )

    sgbm = SGBM()

    while True:
        _, left = cam_l.read()
        _, right = cam_r.read()
        im_l, im_r = rect.apply(left, right)
        disp = sgbm.apply(im_l, im_r)

        cv2.imshow("left", im_l)
        cv2.imshow("right", im_r)
        cv2.imshow("disp", disp)

        cv2.waitKey(1)
