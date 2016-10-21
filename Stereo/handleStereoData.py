#!/usr/bin/python

import rospy

from cv_bridge import CvBridge

import cv2
import numpy as np

from sensor_msgs.msg import PointCloud2, Image
from stereo_msgs.msg import DisparityImage

br = CvBridge()
fgbg = cv2.BackgroundSubtractorMOG2()

def circleArea(r):
    return r*r*3.14159265358979

def identify_blobs(image,processed,size):
    #identified = np.zeros(image.shape, dtype=image.dtype)
    identified = image.copy()

    #BLOB DETECTION ...
    params = cv2.SimpleBlobDetector_Params()
    params.minDistBetweenBlobs = 0

    params.filterByColor = False
    #params.blobColor = 255

    params.filterByArea = True 
    params.minArea = 400.
    params.maxArea = 999999999.

    params.filterByCircularity = False

    params.filterByConvexity = True 
    params.minConvexity = 0.5

    params.filterByInertia = False

    detector = cv2.SimpleBlobDetector(params)

    labels = detector.detect(processed)

    identified = cv2.drawKeypoints(processed,labels,flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    return len(labels), identified


def handleCloud(cloud):
    #print "Height : {}, Width : {}".format(cloud.height, cloud.width)
    pass

def handleImage(msg):
    global br
    im = br.imgmsg_to_cv2(msg)
    cv2.imshow('image', im)

    #im = cv2.fastNlMeansDenoisingColored(im)
    im = cv2.GaussianBlur(im,(3,3),0) 
    #im = cv2.pyrMeanShiftFiltering(im,3,3)

    mask = fgbg.apply(im, 0.5)
    cv2.imshow('mask', mask)

#New code
    if(prvs):
        ret, frame1 = cap.read()
        prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
        hsv = np.zeros_like(frame1)
        hsv[...,1] = 255

        while(1):
            ret, frame2 = cap.read()     
            next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)

            flow = cv2.calcOpticalFlowFarneback(prvs,next, 0.5, 3, 15, 3, 5, 1.2, 0)

            mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
            hsv[...,0] = ang*180/np.pi/2
            hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
            bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

            cv2.imshow('frame2',bgr)
            k = cv2.waitKey(30) & 0xff

            prvs = next



def handleDisparity(msg):
    global br
    im = br.imgmsg_to_cv2(msg.image)
    cv2.normalize(im,im,255.,0.,cv2.NORM_MINMAX)
    im = im.astype(np.uint8)

    k_dilate = np.asarray([
        [.07,.12,.07],
        [.12,.24,.12],
        [.07,.12,.07]
        ],np.float32)

    proc = cv2.dilate(im, k_dilate, iterations = 5) # fill the holes

    #proc = cv2.GaussianBlur(proc,(13,13),0) 

    n, identified = identify_blobs(im,proc,1.)

    print n # print number of blobs

    cv2.imshow('disp', im)
    cv2.imshow('identified', identified)

    cv2.waitKey(1000)
    #print "MIN : {}, MAX : {}".format(disparity.min_disparity, disparity.max_disparity)

def main():
    #dtype, n_channels = br.encoding_to_cvtype2('8UC3')
    rospy.Subscriber("my_stereo/points2", PointCloud2, handleCloud)
    rospy.Subscriber("my_stereo/disparity", DisparityImage, handleDisparity)
    rospy.Subscriber("my_stereo/left/image_rect_color", Image, handleImage)

    win1 = cv2.namedWindow('disp')
    win2 = cv2.namedWindow('image')
    win3 = cv2.namedWindow('identified')

    rospy.init_node("cloudHandler", anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("SHUTTING DOWN")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
