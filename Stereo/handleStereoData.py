#!/usr/bin/python

import rospy

from cv_bridge import CvBridge

import cv2
import numpy as np

from sensor_msgs.msg import PointCloud2, Image
from stereo_msgs.msg import DisparityImage

from blob_detection import *
from background_subtraction import *
from optical_flow import *

br = CvBridge()

opt = None
detector = BlobDetector()
bksub = BackgroundSubtractor()

def handleCloud(cloud):
    # Handle Point Cloud
    #print "Height : {}, Width : {}".format(cloud.height, cloud.width)
    pass

def handleImage(msg):
    global br
    global opt

    im = br.imgmsg_to_cv2(msg)
    cv2.imshow('image', im)

    #im = cv2.fastNlMeansDenoisingColored(im)
    im = cv2.GaussianBlur(im,(3,3),0) 
    #im = cv2.pyrMeanShiftFiltering(im,3,3)

    mask = bksub.apply(im) 
    n, bksub_id = detector.apply(mask)
    cv2.imshow('bksub', bksub_id)

    if(opt != None):
        opt_frame = opt.apply(im)
        opt_frame = cv2.cvtColor(opt_frame, cv2.COLOR_BGR2GRAY)
        _, opt_frame = cv2.threshold(opt_frame, 0, 255, cv2.THRESH_OTSU)
        #cv2.imshow('opt_flow',opt_frame)
        n, opt_id = detector.apply(opt_frame)
        cv2.imshow('opt_flow',opt_id)
    else:
        opt = OpticalFlow(im)

def handleDisparity(msg):
    global br
    im = br.imgmsg_to_cv2(msg.image)
    cv2.normalize(im,im,255.,0.,cv2.NORM_MINMAX)
    im = im.astype(np.uint8)
    im = cv2.GaussianBlur(im,(13,13),0) 
    _, im = cv2.threshold(im, 128, 255, cv2.THRESH_BINARY)

    n, identified = detector.apply(im)

    print n # print number of blobs

    #cv2.imshow('disp', im)
    cv2.imshow('disp-blobs', identified)

    cv2.waitKey(1)
    #print "MIN : {}, MAX : {}".format(disparity.min_disparity, disparity.max_disparity)

def main():
    #dtype, n_channels = br.encoding_to_cvtype2('8UC3')
    rospy.Subscriber("my_stereo/points2", PointCloud2, handleCloud)
    rospy.Subscriber("my_stereo/disparity", DisparityImage, handleDisparity)
    rospy.Subscriber("my_stereo/left/image_rect_color", Image, handleImage)

    # Instantiate all the windows
    #win1 = cv2.namedWindow('disp')
    win2 = cv2.namedWindow('image')
    win3 = cv2.namedWindow('disp-blobs')
    win4 = cv2.namedWindow('opt_flow')

    rospy.init_node("cloudHandler", anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("SHUTTING DOWN")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
