#!/usr/bin/python

import rospy

from cv_bridge import CvBridge

import cv2
import numpy as np

from sensor_msgs.msg import PointCloud2
from stereo_msgs.msg import DisparityImage

br = CvBridge()

def identify_blobs(image,processed,size):
    identified = image.copy()

    #BLOB DETECTION ...
    params = cv2.SimpleBlobDetector_Params()
    params.minDistBetweenBlobs = 0

    params.filterByColor = True 
    params.blobColor = 255

    params.filterByArea = True 
    params.minArea = circleArea(size) * 0.3 
    params.maxArea = circleArea(size) * 2.0

    params.filterByCircularity = False

    params.filterByConvexity = True 
    params.minConvexity = 0.5

    params.filterByInertia = False

    detector = cv2.SimpleBlobDetector_create(params)

    labels = detector.detect(processed)

    cv2.drawKeypoints(identified,labels,identified,color=(255,0,0),flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    return len(labels), identified


def handleCloud(cloud):
    #print "Height : {}, Width : {}".format(cloud.height, cloud.width)
    pass

def handleDisparity(msg):
    global br
    im = br.imgmsg_to_cv2(msg.image)
    cv2.normalize(im,im,1.,0.,cv2.NORM_MINMAX)
    n, identified = identify_blobs(im,im,20.)
    cv2.imshow('image', im)
    cv2.imshow('identified', identified)
    cv2.waitKey(3)
    #print "MIN : {}, MAX : {}".format(disparity.min_disparity, disparity.max_disparity)

def main():
    #dtype, n_channels = br.encoding_to_cvtype2('8UC3')
    rospy.Subscriber("my_stereo/points2", PointCloud2, handleCloud)
    rospy.Subscriber("my_stereo/disparity", DisparityImage, handleDisparity)

    win = cv2.namedWindow('image')

    rospy.init_node("cloudHandler", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("SHUTTING DOWN")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
