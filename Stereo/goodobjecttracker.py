#!/usr/bin/python

#### Object Tracking Code ####
# Lydia Zuehsow, Olin College of Engineering Oct 2016
# This code takes in a single video input and tracks significant objects, identifying them with a green circle. 
# Objects are determined to be significant based on the area of the contour that they produce.
# The background is identified unpon startup, and objects are tracked by their change in position relative to that static image.

import cv2
import imutils
import numpy as np
import argparse
from background_subtraction import BackgroundSubtractor

bksub = BackgroundSubtractor()

class Target(object):
	def __init__(self):
		self.camera = cv2.VideoCapture(0)
		# construct the argument parser and parse the arguments
		ap = argparse.ArgumentParser()
		ap.add_argument("-v", "--video", help="path to the video file")
		
		ap.add_argument("-a", "--min-area", type=int, default=1500, help="minimum area size")
		self.args = vars(ap.parse_args())

		self.count = 0
		# self.firstframes = []
		self.firstframe = None
		self.thresh = None

		self.yellowLower = (20, 100, 100)
		self.yellowUpper = (30, 255, 255)

		self.redLower = (0, 100, 100)
		self.redUpper = (30, 255, 255)
		self.redLower1 = (160, 100, 100)
		self.redUpper1 = (180, 255, 255)

		self.greenLower = (0,100,100)
		self.greenUpper = (80,255,255)

		self.blueLower = (90,100,100)
		self.blueUpper = (125,255,255)

		self.Lower = [self.yellowLower,self.greenLower]
		self.Upper =[self.yellowUpper,self.greenUpper]

		self.objectlist = []#[[0,(0,0),'Null']] # Radius, XY coordinates.

class Find_Color(object):
	def __init__(self, frame):
		self.blurred = cv2.GaussianBlur(frame, (21, 21), 0)
		self.objcolor = "Null"

	def findobject(self):
		if counter <= 15:
			print "Calib"
		if counter >= 10:
			# t.objectlist.append([radius,(center[0],center[1])])

			temp = False
			objnum = 0

			for obj in t.objectlist:
				objrad = obj[0]

				if ((radius - 25.0) <= objrad) and ((radius + 25.0) >= objrad):
					print 'Found object:' , objnum
					# print 'List is now: ' , t.objectlist
					temp = True

				objnum += 1
				
			if temp == False:
				t.objectlist.append([radius,(cX, cY),fc.objcolor])
				print 'Added object to list:' , [radius,(int(center[0]), int(center[1])),str(fc.objcolor)]
				print fc.objcolor
				print 'List is now: ' , t.objectlist



if __name__=="__main__":

	t = Target()
	counter = 0

	while True:
		(grabbed, frame) = t.camera.read()

		# if the frame could not be grabbed, then we have reached the end
		# of the video

		# resize the frame, convert it to grayscale, and blur it
		#t.frame = imutils.resize(frame, width=500)
		t.frame = frame
		hsv = cv2.cvtColor(t.frame, cv2.COLOR_BGR2HSV)
		fc = Find_Color(hsv)
		
		gray = cv2.cvtColor(t.frame, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (21, 21), 0)
		# if the first frame is None, initialize it
		if t.frame is None:
			t.frame = gray
			continue
		else:
			if t.count == 0:
				t.firstframe = gray
				t.count += 1

			if t.thresh is not None:
				(cnts, _) = cv2.findContours(cv2.GaussianBlur(t.thresh.copy(), (25,25), 0), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
				# loop over the contours

				for c in cnts:
					# if the contour is too small, ignore it
					if cv2.contourArea(c) < t.args["min_area"]:
						continue
					else:
						(center,radius) = cv2.minEnclosingCircle(c)
						if radius >= 50:
							M = cv2.moments(c)
							cX = int(M["m10"] / M["m00"])
							cY = int(M["m01"] / M["m00"])
							cv2.circle(t.frame, (cX, cY), 7, (255, 255, 255), -1)
							cv2.circle(t.frame, (cX, cY), int(radius), (255,255,255), 5)

							# print "radius: " , radius
							
							# cv2.circle(t.frame, (int(center[0]), int(center[1])), 5, (0,255,0), -1)
							# cv2.circle(t.frame, (int(center[0]), int(center[1])), int(radius), (0,255,0), 5)
							# print "center", center
							
							cv2.imshow('blr', fc.blurred)

							# color = fc.blurred[int(center[1]),int((center[0]))]
							# print cX, cY

							if (cX >= 480):
								cX = 479
							elif (cX <= 0):
								cX = 1


							color = fc.blurred[(cX,cY)]
							print color[0]
							# print radius

							# if ((color[0] <= t.yellowUpper[0]) and (color[0] >= t.yellowLower[0])):
							# 	# print 'Found yellow object'
							# 	# print center
							# 	# print color
							# 	fc.objcolor = "Yellow"
							# 	fc.findobject()
							# 	counter +=1 
							if (((color[0] <= t.redUpper[0]) and (color[0] >= t.redLower[0])) or ((color[0] <= t.redUpper1[0]) and (color[0] >= t.redLower1[0]))):
								fc.objcolor = "Red"
								fc.findobject()
								counter +=1 
							if ((color[0] <= t.greenUpper[0]) and (color[0] >= t.greenLower[0])):
								fc.objcolor = "Green"
								fc.findobject()
								counter +=1
							if ((color[0] <= t.blueUpper[0]) and (color[0] >= t.blueLower[0])):
								fc.objcolor = "Blue"
								fc.findobject()
								counter +=1 

			cv2.imshow("Camera", t.frame)

		if t.firstframe is not None:
			frameDelta = cv2.absdiff(t.firstframe, gray)
			t.thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
			t.thresh = cv2.dilate(t.thresh, None, iterations=7)

			(cnts, _) = cv2.findContours(t.thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                        #t.thresh = bksub.apply(gray)

			cv2.imshow("Static Frame", t.thresh)

		if np.average(t.thresh) > 200:
			t.firstframe = gray

		key = cv2.waitKey(1) & 0xFF
		# if the `q` key is pressed, break from the lop
		if key == ord("q"):
			print t.objectlist
			break
		if key == ord("r"):
			t.firstframe = gray
