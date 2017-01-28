#!/usr/bin/python

import os

# ROS
import rospy
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
from better_objecttracker import Object, ObjectTracker

## Global Variables
opt = None
detector = BlobDetector()
bksub = BackgroundSubtractor()
sgbm = SGBM()

k_dilate = cv2.getStructuringElement(cv2.MORPH_DILATE, (7,7),(3,3))
k_erode = cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3))
k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))


def neighborhood(x,y,n):
    #v = np.round(np.linspace(-n/2.,n/2.,n))
    v = np.linspace(-n/2.,n/2.,n,dtype=np.int)
    xs,ys = x+v,y+v
    return xs,ys

class ObjectManager(object):
    def __init__(self):
        self.current_index = 0
        self.objects = {} # dictionary of 'name, list of object'
    def add(self, new_object):
        if len(self.objects) > 0:
            cv2.imshow('object', self.objects.values()[-1][0].img)

        obj_found = False
        for k,v in self.objects.iteritems():
            l_v = len(v)
            for obj in np.random.choice(v,min(l_v,10)):
                if obj == new_object: #"same object"
                    v.append(new_object)
                    obj_found = True
                    break
            if obj_found:
                break
        else:
            new_name = 'object_{}'.format(self.current_index)
            print new_name
            self.objects[new_name] = [new_object]
            self.current_index += 1
            # no match

def projectDisparityTo3d(x,y,Q,d):
    x,y,z = (Q[0,0]*x + Q[0,3], Q[1,1]*y + Q[1,3], Q[2,3])
    w = Q[3,2]*d + Q[3,3]
    return (x/w, y/w, z/w)

def handle_disp(im_l, im_r, Q):
    raw_disp = sgbm.apply(im_l, im_r)
    raw_disp = cv2.dilate(raw_disp, k_dilate, iterations=5)
    raw_disp = cv2.GaussianBlur(raw_disp,(3,3),0) 

    disp = cv2.normalize(raw_disp,None,0,255,cv2.NORM_MINMAX).astype(np.uint8)
    disp = cv2.GaussianBlur(disp,(13,13),0) 
    _, disp = cv2.threshold(disp, 128, 255, cv2.THRESH_BINARY)
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
    mask = cv2.dilate(mask, k_dilate, iterations=2)
    mask = cv2.GaussianBlur(mask,(3,3),0)
    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,k_close)

    return mask #detector.apply(mask), mask

def fill_holes(mask):
    th, im_th = cv2.threshold(mask, 220, 255, cv2.THRESH_BINARY_INV);
    # Copy the thresholded image.
    im_floodfill = im_th.copy()
    # Mask used to flood filling.
    # Notice the size needs to be 2 pixels than the image.
    h, w = im_th.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
    # Floodfill from point (0, 0)
    cv2.floodFill(im_floodfill, mask, (0,0), 255);
    # Invert floodfilled image
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
    # Combine the two images to get the foreground.
    filled = im_th | im_floodfill_inv
    return filled

def demo():
    rospy.init_node('object_tracker')
    # initialize rectifier
    rospack = rospkg.RosPack()
    pkg_root = rospack.get_path('edwin')

    rectifier = Rectifier(
            param_l = os.path.join(pkg_root, 'Stereo/camera_info/left_camera.yaml'),
            param_r = os.path.join(pkg_root, 'Stereo/camera_info/right_camera.yaml')
            )

    cam_l = cv2.VideoCapture(1)
    cam_r = cv2.VideoCapture(2)

    cnt = 0

    last_cropped = None
    matcher = Matcher()
    tracker = ObjectTracker()
    manager = ObjectManager()

    tracker.set_target('medium','blue')

    for i in range(10):
        cam_l.read()
        cam_r.read()

    while not rospy.is_shutdown():
        _, left = cam_l.read()
        _, right = cam_r.read()
        im_l, im_r = rectifier.apply(left, right)

        im_disp, raw_disp = handle_disp(im_l, im_r, rectifier.Q)

        raw_dist = cv2.reprojectImageTo3D((raw_disp/16.).astype(np.float32), rectifier.Q, handleMissingValues=True)
        #dist = np.linalg.norm(dist,axis=2)
        dist = raw_dist[:,:,2]
        _,dist = cv2.threshold(dist, 1.0, 1.0, cv2.THRESH_BINARY_INV)
        dist = (dist*255).astype(np.uint8)
        dist = cv2.erode(dist, k_erode)

        #cv2.imshow('im_disp', im_disp)
        cv2.imshow('dist', dist)

        # Now Apply Blur ...
        blur = cv2.GaussianBlur(im_l,(3,3),0) 
        
        im_opt = handle_opt(blur)
        im_bksub = handle_bksub(blur)

        # rudimentary, combine detection data
        im_t = cv2.addWeighted(dist, 0.5, im_opt, 0.5, 0)
        im_comb = cv2.addWeighted(im_t, 2./3, im_bksub, 1./3, 0)


        #cv2.imshow("raw_disp", raw_disp)
        _, im_comb = cv2.threshold(im_comb, 128, 255, cv2.THRESH_BINARY)
        cv2.imshow("im_comb", im_comb)
        cv2.imshow("im_opt", im_opt)
        cv2.imshow("im_bksub", im_bksub)

        #rect = detector.apply(im_comb)
        identified = im_l.copy()

        #dist = cv2.reprojectImageTo3D(raw_disp, rectifier.Q, handleMissingValues=True) # for all of disparity map
        #print dist
        #target_pos, target_img = tracker.apply(im_comb, im_l, dist)
        #print 'target_pos', target_pos
        #if target_img != None:
        #    cv2.imshow('target_img', target_img)
        #cv2.circle(identified, target_pos, 10, (255,255,255), 2)

        ret = detector.apply(im_comb)
        if ret != None:
            x,y,w,h,m,a = ret
            if w*h > 100*100: # bigger than 100x100 px
                cv2.rectangle(identified, (x,y), (x+w, y+h), (255,0,0),2)
                cropped = im_l[y:y+h,x:x+w]

                cX = int(m["m10"] / m["m00"])
                cY = int(m["m01"] / m["m00"])
                #cX,cY = 320,240

                cv2.circle(identified, (cX, cY), 10, (255,255,255), 2)

                xs,ys = neighborhood(cX,cY,5)
                ds = []
                for x in xs:
                    for y in ys:
                        #dist = projectDisparityTo3d(x, y, rectifier.Q, raw_disp[cY,cX]/16.) # for single point
                        #dist = np.linalg.norm(dist)
                        d = raw_dist[y,x,2]
                        if 0 < d and d < 5: # within reasonable range
                            ds.append(d)

                color = identified[cY,cX]
                d = np.median(ds)
                o = Object(cropped, color,(cX,cY),a*d*d)# proportional to d**2, need scaling factor k(undetermined)
                manager.add(o)
                print 'l', len(manager.objects)

                #if last_cropped != None:
                #    same, match_frame= matcher.match(last_cropped, cropped, draw=True)
                #    print same
                #    cv2.imshow("match", match_frame)
                #last_cropped = cropped

                ### WRITE TO FILE ###
                #cv2.imwrite("data_%d.png" % cnt, cropped)
                cnt += 1

        cv2.imshow("identified", identified)
        cv2.waitKey(1)

    cv2.destroyAllWindows()

#def generate_dataset():
#
#    ## Initialize Rectifier
#    rospack = rospkg.RosPack()
#    pkg_root = rospack.get_path('edwin')
#
#    cam_l = cv2.VideoCapture(2)
#    cam_r = cv2.VideoCapture(1)
#
#    rectifier = Rectifier(
#            param_l = os.path.join(pkg_root, 'Stereo/camera_info/left_camera.yaml'),
#            param_r = os.path.join(pkg_root, 'Stereo/camera_info/right_camera.yaml')
#            )
#
#    k_dilate = np.asarray([
#            [.07,.12,.07],
#            [.12,.24,.12],
#            [.07,.12,.07]
#            ],np.float32)
#
#    cnt = 0
#    last_cropped = None
#    while True:
#        _, left = cam_l.read()
#        _, right = cam_r.read()
#        im_l, im_r = rectifier.apply(left, right)
#
#        im_disp, raw_disp = handle_disp(im_l, im_r, rectifier.Q)
#
#        #print pcl[:,:,2]
#        #cv2.imshow('pcl', pcl)
#
#        # Now Apply Blur ...
#        blur = cv2.GaussianBlur(im_l,(3,3),0) 
#        
#        im_opt = handle_opt(blur)
#        im_bksub = handle_bksub(blur)
#
#        # rudimentary, combine detection data
#        im_t = cv2.addWeighted(im_disp, 0.5, im_opt, 0.5, 0)
#        im_comb = cv2.addWeighted(im_t, 2./3, im_bksub, 1./3, 0)
#
#        #im_comb = cv2.GaussianBlur(im_comb,(31,31),0) 
#        #im_comb = cv2.GaussianBlur(im_comb,(31,31),0) 
#        #im_comb = cv2.GaussianBlur(im_comb,(31,31),0) 
#        #cv2.multiply(im_comb, im_comb, im_comb, 1./255)
#        #_, im_comb = cv2.threshold(im_comb, 30, 255, cv2.THRESH_BINARY)
#
#        #im_comb = cv2.dilate(im_comb, k_dilate, iterations = 3) 
#
#        cv2.imshow("im_disp", im_disp)
#        #cv2.imshow("im_opt", im_opt)
#        #cv2.imshow("im_bksub", im_bksub)
#        #cv2.imshow("im_comb", im_comb)
#        rect = detector.apply(im_comb)
#
#        identified = im_l.copy()
#
#        #dist = cv2.reprojectImageTo3D(raw_disp, rectifier.Q, handleMissingValues=True) # for all of disparity map
#        if rect != None:
#            x,y,w,h,m = rect
#
#            if w*h < 100000:
#                cv2.rectangle(identified, (x,y), (x+w, y+h), (255,0,0),2)
#                cropped = im_l[y:y+h,x:x+w]
#                #cv2.imshow("cropped", cropped)
#
#                cX = int(m["m10"] / m["m00"])
#                cY = int(m["m01"] / m["m00"])
#                cv2.circle(identified, (cX, cY), 10, (255,255,255), 2)
#                print projectDisparityTo3d(cX, cY, rectifier.Q,  raw_disp[cY,cX]) # for single point
#                #cv2.imwrite("data_%d.png" % cnt, cropped)
#                cnt += 1
#
#        cv2.imshow("identified", identified)
#
#        cv2.waitKey(1)
#
#
#    cv2.destroyAllWindows()

def main():
    demo()

if __name__ == '__main__':
    main()
