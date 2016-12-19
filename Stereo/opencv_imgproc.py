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
from match import Matcher

## Global Variables
opt = None
detector = BlobDetector()
bksub = BackgroundSubtractor()
sgbm = SGBM()

def projectDisparityTo3d(x,y,Q,d):
    x,y,z = (Q[0,0]*x + Q[0,3], Q[1,1]*y + Q[1,3], Q[2,3])
    w = Q[3,2]*d + Q[3,3]
    return (x/w, y/w, z/w)

def handle_disp(im_l, im_r, Q):
    raw_disp = sgbm.apply(im_l, im_r)

    disp = cv2.normalize(raw_disp,None,0,255,cv2.NORM_MINMAX).astype(np.uint8)
    disp = cv2.GaussianBlur(disp,(13,13),0) 
    #_, disp = cv2.threshold(disp, 128, 255, cv2.THRESH_BINARY)
    return disp, raw_disp #detector.apply(disp), disp

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

def demo():
    # returns raw image, distance map, and mask of where objects are not

    # initialize rectifier
    rospack = rospkg.RosPack()
    pkg_root = rospack.get_path('edwin')

    rectifier = Rectifier(
            param_l = os.path.join(pkg_root, 'Stereo/camera_info/left_camera.yaml'),
            param_r = os.path.join(pkg_root, 'Stereo/camera_info/right_camera.yaml')
            )

    cam_l = cv2.VideoCapture(2)
    cam_r = cv2.VideoCapture(1)

    cnt = 0

    last_cropped = None
    matcher = Matcher()

    while True:
        _, left = cam_l.read()
        _, right = cam_r.read()
        im_l, im_r = rectifier.apply(left, right)

        im_disp, raw_disp = handle_disp(im_l, im_r, rectifier.Q)

        # Now Apply Blur ...
        blur = cv2.GaussianBlur(im_l,(3,3),0) 
        
        im_opt = handle_opt(blur)
        im_bksub = handle_bksub(blur)

        # rudimentary, combine detection data
        im_t = cv2.addWeighted(im_disp, 0.5, im_opt, 0.5, 0)
        im_comb = cv2.addWeighted(im_t, 2./3, im_bksub, 1./3, 0)

        _, im_comb = cv2.threshold(im_comb, 30, 255, cv2.THRESH_BINARY)
        cv2.imshow("im_disp", im_disp)
        #cv2.imshow("im_opt", im_opt)
        #cv2.imshow("im_bksub", im_bksub)
        cv2.imshow("im_comb", im_comb)

        #rect = detector.apply(im_comb)

        identified = im_l.copy()

        dist = cv2.reprojectImageTo3D(raw_disp, rectifier.Q, handleMissingValues=True) # for all of disparity map

        rect = detector.apply(im_comb)
        if rect != None:
            x,y,w,h,m = rect
            if w*h < 100000:
                cv2.rectangle(identified, (x,y), (x+w, y+h), (255,0,0),2)
                cropped = im_l[y:y+h,x:x+w]

                cX = int(m["m10"] / m["m00"])
                cY = int(m["m01"] / m["m00"])
                cv2.circle(identified, (cX, cY), 10, (255,255,255), 2)
                #print projectDisparityTo3d(cX, cY, rectifier.Q,  raw_disp[cY,cX]) # for single point

                if last_cropped != None:
                    same, match_frame= matcher.match(last_cropped, cropped, draw=True)
                    print same
                    cv2.imshow("match", match_frame)
                last_cropped = cropped


                #cv2.imwrite("data_%d.png" % cnt, cropped)
                cnt += 1
        #obj = lydia(im_l,dist,im_comb) # lydia's code here

        cv2.imshow("identified", identified)

        cv2.waitKey(1)


    cv2.destroyAllWindows()

def generate_dataset():

    ## Initialize Rectifier
    rospack = rospkg.RosPack()
    pkg_root = rospack.get_path('edwin')

    cam_l = cv2.VideoCapture(2)
    cam_r = cv2.VideoCapture(1)

    rectifier = Rectifier(
            param_l = os.path.join(pkg_root, 'Stereo/camera_info/left_camera.yaml'),
            param_r = os.path.join(pkg_root, 'Stereo/camera_info/right_camera.yaml')
            )

    k_dilate = np.asarray([
            [.07,.12,.07],
            [.12,.24,.12],
            [.07,.12,.07]
            ],np.float32)

    cnt = 0
    last_cropped = None
    while True:
        _, left = cam_l.read()
        _, right = cam_r.read()
        im_l, im_r = rectifier.apply(left, right)

        im_disp, raw_disp = handle_disp(im_l, im_r, rectifier.Q)

        #print pcl[:,:,2]
        #cv2.imshow('pcl', pcl)

        # Now Apply Blur ...
        blur = cv2.GaussianBlur(im_l,(3,3),0) 
        
        im_opt = handle_opt(blur)
        im_bksub = handle_bksub(blur)

        # rudimentary, combine detection data
        im_t = cv2.addWeighted(im_disp, 0.5, im_opt, 0.5, 0)
        im_comb = cv2.addWeighted(im_t, 2./3, im_bksub, 1./3, 0)

        #im_comb = cv2.GaussianBlur(im_comb,(31,31),0) 
        #im_comb = cv2.GaussianBlur(im_comb,(31,31),0) 
        #im_comb = cv2.GaussianBlur(im_comb,(31,31),0) 
        #cv2.multiply(im_comb, im_comb, im_comb, 1./255)
        #_, im_comb = cv2.threshold(im_comb, 30, 255, cv2.THRESH_BINARY)

        #im_comb = cv2.dilate(im_comb, k_dilate, iterations = 3) 

        cv2.imshow("im_disp", im_disp)
        #cv2.imshow("im_opt", im_opt)
        #cv2.imshow("im_bksub", im_bksub)
        #cv2.imshow("im_comb", im_comb)
        rect = detector.apply(im_comb)

        identified = im_l.copy()

        #dist = cv2.reprojectImageTo3D(raw_disp, rectifier.Q, handleMissingValues=True) # for all of disparity map
        if rect != None:
            x,y,w,h,m = rect

            if w*h < 100000:
                cv2.rectangle(identified, (x,y), (x+w, y+h), (255,0,0),2)
                cropped = im_l[y:y+h,x:x+w]
                #cv2.imshow("cropped", cropped)

                cX = int(m["m10"] / m["m00"])
                cY = int(m["m01"] / m["m00"])
                cv2.circle(identified, (cX, cY), 10, (255,255,255), 2)
                print projectDisparityTo3d(cX, cY, rectifier.Q,  raw_disp[cY,cX]) # for single point
                #cv2.imwrite("data_%d.png" % cnt, cropped)
                cnt += 1

        cv2.imshow("identified", identified)

        cv2.waitKey(1)


    cv2.destroyAllWindows()

def main():
    demo()

if __name__ == '__main__':
    main()
