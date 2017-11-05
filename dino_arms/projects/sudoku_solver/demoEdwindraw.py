#!/usr/bin/env python
import rospy
import math
# import st
import numpy as np
from std_msgs.msg import String
from edwin.msg import *
import time
import getRedCircles

'''
run robot_write.launch
also run arm_node.py first
'''
pixelError = 5
tiltError = 0.3

class movementControl:
	moveMult = 5
	tiltMult = 5
	rectangle = None

	def __init__(self,pos = None):
		if pos is None:
			self.pos = (-500,5400,-835)
		else:
			self.pos = pos
		self.wristPos = 1000
		self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)


	def move(self):
		msg = "data: move_to:: " + str(pos[0]) + ", " + str(pos[1]) + ", " + str(pos[1]) + ", " +str(0)
		print "sending: ", msg
		self.arm_pub.publish(msg)

	def tilt(self):
		msg = "data: rotate_wrist:: " + str(self.wristPos)
		print "sending: ", msg
		self.arm_pub.publish(msg)

	def findPoint(circleObj, method):
		xDiff,yDiff = method(changePoints(circleObj.permCorners))
		while abs(xDiff) > pixelError or abs(yDiff) > pixelError:
			self.pos[0] += int(xDiff * self.moveMult)
			self.pos[1] += int(yDiff * self.moveMult)
			self.move()
			self.straightenRect(circleObj)
			circleObj.runOnce()
			xDiff,yDiff = method(changePoints(circleObj.permCorners))
		return pos[0],pos[1]

	def findCenter(circleObj):
		return findPoint(circleObj.getCenter)

	def findLeft(circleObj):
		return findPoint(circleObj.getLeft)

	def findTop(circleObj):
		return findPoint(circleObj.getTop)

	def straightenRect(circleObj):
		#positive result form circleObj means turn right
		tiltVal = circleObj.getRotation(changePoints(circleObj.permCorners))
		while abs(tiltVal) > tiltError:
			self.wristPos -= tiltVal * tiltMult
			self.tilt()
			circleObj.runOnce()
			tiltVal = circleObj.getRotation(changePoints(circleObj.permCorners))

	def getDims(circleObj):
		centerx, centery = findCenter(circleObj)
		topx, topy = findTop(circleObj)
		leftx, lefty = findLeft(circleObj)


def z_calculation(input_y):
	scaler = -735 - int((input_y - 4000)/9.4)
	return scaler


def run():
	rospy.init_node('arm_tester', anonymous=True)
	pub = rospy.Publisher('write_cmd', Edwin_Shape, queue_size=10)
	arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
	time.sleep(2)
	print "starting"

	# while not rospy.is_shutdown():
	msg = Edwin_Shape()


	msg.shape = "123456"
	msg.x = -500
	msg.y = 5700
	msg.z = -835
	pub.publish(msg)
	time.sleep(5)

	msg.shape = "7890!?"
	msg.x = -500
	msg.y = 5400
	msg.z = -835
	pub.publish(msg)
	time.sleep(5)
	#
	# msg.shape = "abcdef"
	# msg.x = -500
	# msg.y = 5700
	# msg.z = -790
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "ghijkl"
	# msg.x = -500
	# msg.y = 5400
	# msg.z = -790
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "mnopqr"
	# msg.x = -500
	# msg.y = 5100
	# msg.z = -780
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "stuvwx"
	# msg.x = -500
	# msg.y = 4800
	# msg.z = -770
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "yz"
	# msg.x = -500
	# msg.y = 4500
	# msg.z = -770
	# pub.publish(msg)
	# time.sleep(5)

if __name__ == '__main__':
	run()
