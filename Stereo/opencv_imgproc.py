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
sgbm = SGBM()


def handle_disp(im_l, im_r):
    disp = sgbm.apply(im_l, im_r)
    disp = cv2.GaussianBlur(disp,(13,13),0) 
    _, disp = cv2.threshold(disp, 128, 255, cv2.THRESH_BINARY)

    return disp #detector.apply(disp), disp

def handle_opt(im):
    global opt
    if opt != None:
        opt_frame = opt.apply(im)
        opt_frame = cv2.cvtColor(opt_frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("opt_flow", opt_frame)
        # activation should be at least greater than 30
        thr, opt_frame = cv2.threshold(opt_frame, 50, 255, cv2.THRESH_BINARY)
        return opt_frame #detector.apply(opt_frame), opt_frame
    else:
        opt = OpticalFlow(im)
        opt_frame = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        return opt_frame #([], opt_frame), opt_frame

def handle_bksub(im):
    mask = bksub.apply(im) 
    return mask #detector.apply(mask), mask

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

    k_dilate = np.asarray([
            [.07,.12,.07],
            [.12,.24,.12],
            [.07,.12,.07]
            ],np.float32)


    while True:
        _, left = cam_l.read()
        _, right = cam_r.read()
        im_l, im_r = rect.apply(left, right)

        im_disp = handle_disp(im_l, im_r)


        # Now Apply Blur ...
        im_l = cv2.GaussianBlur(im_l,(3,3),0) 
        
        im_opt = handle_opt(im_l)
        im_bksub = handle_bksub(im_l)

        im_t = cv2.addWeighted(im_disp, 0.5, im_opt, 0.5, 0)
        im_comb = cv2.addWeighted(im_t, 2./3, im_bksub, 1./3, 0)

        #im_comb = cv2.GaussianBlur(im_comb,(31,31),0) 
        #im_comb = cv2.GaussianBlur(im_comb,(31,31),0) 
        #im_comb = cv2.GaussianBlur(im_comb,(31,31),0) 
        #cv2.multiply(im_comb, im_comb, im_comb, 1./255)
        #_, im_comb = cv2.threshold(im_comb, 30, 255, cv2.THRESH_BINARY)

        #im_comb = cv2.dilate(im_comb, k_dilate, iterations = 3) 

        #cv2.imshow("im_disp", im_disp)
        #cv2.imshow("im_opt", im_opt)
        #cv2.imshow("im_bksub", im_bksub)
        #cv2.imshow("im_comb", im_comb)
        labels, identified = detector.apply(im_comb)
        cv2.imshow("identified", identified)

        cv2.waitKey(1)


    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
