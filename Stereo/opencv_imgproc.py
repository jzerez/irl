#!/usr/bin/python

import os

# ROS
import rospkg

# OPENCV
import cv2

# ROS-OPENCV
from cv_bridge import CvBridge

import numpy as np

# IMG PROC
from blob_detection import *
from background_subtraction import *
from optical_flow import *

# IMG Rectification
from image_rectification import Rectifier, SGBM

## Global Variables
opt = None
detector = BlobDetector()
bksub = BackgroundSubtractor()

def handle_disp(im):
    im = cv2.GaussianBlur(im,(13,13),0) 
    _, im = cv2.threshold(im, 128, 255, cv2.THRESH_BINARY)
    return detector.apply(im)

def handle_opt(im):
    global opt
    if opt != None:
        opt_frame = opt.apply(im)
        #opt_frame = cv2.cvtColor(opt_frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("opt_flow", opt_frame)
        # activation should be at least greater than 30
        thr, opt_frame = cv2.threshold(opt_frame, 50, 255, cv2.THRESH_BINARY)
        return detector.apply(opt_frame)
    else:
        opt = OpticalFlow(im)
        return [], im

def handle_bksub(im):
    mask = bksub.apply(im) 
    return detector.apply(mask)

def main():

    ## Initialize Rectifier
    rospack = rospkg.RosPack()
    pkg_root = rospack.get_path('edwin')

    cam_l = cv2.VideoCapture(1)
    cam_r = cv2.VideoCapture(2)

    rect = Rectifier(
            param_l = os.path.join(pkg_root, 'Stereo/camera_info/left_camera.yaml'),
            param_r = os.path.join(pkg_root, 'Stereo/camera_info/right_camera.yaml')
            )

    ## 
    ## Initialize SGBM
    sgbm = SGBM()

    while True:
        _, left = cam_l.read()
        _, right = cam_r.read()
        im_l, im_r = rect.apply(left, right)

        disp = sgbm.apply(im_l, im_r)
        l_disp, id_disp = handle_disp(disp)

        # Now Apply Blur ...
        im_l = cv2.GaussianBlur(im_l,(3,3),0) 
        
        l_opt, id_opt = handle_opt(im_l)
        l_bksub, id_bksub = handle_bksub(im_l)

        cv2.imshow("disp", disp)
        cv2.imshow("id_disp", id_disp)
        cv2.imshow("id_opt", id_opt)
        cv2.imshow("id_bksub", id_bksub)

        cv2.waitKey(1)


    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
