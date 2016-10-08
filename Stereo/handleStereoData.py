#!/usr/bin/python

import rospy

from cv_bridge import CvBridge

import cv2
import numpy as np

from sensor_msgs.msg import PointCloud2
from stereo_msgs.msg import DisparityImage

br = CvBridge()

def handleCloud(cloud):
    #print "Height : {}, Width : {}".format(cloud.height, cloud.width)
    pass

def handleDisparity(msg):
    global br
    im = br.imgmsg_to_cv2(msg.image)
    cv2.normalize(im,im,1.,0.,cv2.NORM_MINMAX)
    cv2.imshow('image', im)
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
