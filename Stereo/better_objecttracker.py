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
	def __init__(self, umod_frame, dist, bkg_mask):
		self.frame = umod_frame
		(rows,cols,color) = dim.shape
		self.dim = [rows,cols]
		self.dist = dist
		# construct the argument parser and parse the arguments
		ap = argparse.ArgumentParser()
		
		ap.add_argument("-a", "--min-area", type=int, default=1500, help="minimum area size")
		self.args = vars(ap.parse_args())

		self.count = 0
		self.bkg_mask = bkg_mask

		self.redLower = (0, 100, 100)
		self.redUpper = (30, 255, 255)
		self.redLower1 = (160, 100, 100)
		self.redUpper1 = (180, 255, 255)

		self.greenLower = (0,100,100)
		self.greenUpper = (80,255,255)

		self.blueLower = (90,100,100)
		self.blueUpper = (125,255,255)

		# self.Lower = [self.yellowLower,self.greenLower]
		# self.Upper =[self.yellowUpper,self.greenUpper]

		self.objectlist = []#[[0,(0,0),'Null']] # Radius, XY coordinates.

class Find_Color(object):
	def __init__(self, frame):
		self.blurred = cv2.GaussianBlur(frame, (21, 21), 0)
		self.objcolor = "Null"

	def findobject(self):
		temp = False
		objnum = 0

		for obj in t.objectlist:
			objarea = obj[0]

			if ((area - 25.0) <= objarea) and ((area + 25.0) >= objarea):
				print 'Found object:' , objnum
				# print 'List is now: ' , t.objectlist
				temp = True

			objnum += 1
			
		if temp == False:
			t.objectlist.append([area,(cX, cY),fc.objcolor])
			print 'Added object to list:' , [area,(int(center[0]), int(center[1])),str(fc.objcolor)]
			print fc.objcolor
			print 'List is now: ' , t.objectlist

	def colorcompare(self, color):
		if (((color[0] <= t.redUpper[0]) and (color[0] >= t.redLower[0])) or ((color[0] <= t.redUpper1[0]) and (color[0] >= t.redLower1[0]))):
			fc.objcolor = "Red"
			fc.findobject()
		if ((color[0] <= t.greenUpper[0]) and (color[0] >= t.greenLower[0])):
			fc.objcolor = "Green"
			fc.findobject()
		if ((color[0] <= t.blueUpper[0]) and (color[0] >= t.blueLower[0])):
			fc.objcolor = "Blue"
			fc.findobject()

	def areacalc(self, contour, dist):
		[xdim,ydim] = t.dim

		x,y,w,h = cv2.boundingRect(contour)
		centerx = x+(0.5*w)
		centery = y+(0.5*h)

		xmin_theta = x * (180.0/xdim)
		ymin_theta = y * (180.0/ydim)
		xmax_theta = (x+w) * (180.0/xdim)
		ymax_theta = (y+h) * (180.0/ydim)

		xmin_width = np.tan(xmin_theta) * dist
		ymin_length = np.tan(ymin_theta) * dist
		xmax_width = np.tan(xmax_theta) * dist
		ymax_length = np.tan(ymax_theta) * dist

		xwidth = xmax_length - xmin_length
		ylength = ymax_length - ymin_length

		area = xwidth * ylength
		return area


if __name__=="__main__":

	t = Target(umod_frame, dist, bkg_mask)

	while True:
		
		hsv = cv2.cvtColor(t.frame, cv2.COLOR_BGR2HSV)
		fc = Find_Color(hsv)
		
		# if the first frame is None, initialize it
		if t.bkg_mask is not None:
			(cnts, _) = cv2.findContours(cv2.GaussianBlur(t.bkg_mask.copy(), (25,25), 0), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
			# loop over the contours

			for c in cnts:
				# if the contour is too small, ignore it
				if cv2.contourArea(c) < t.args["min_area"]:
					continue
				else:
					(center,radius) = cv2.minEnclosingCircle(c)
					area = fc.areacalc(t.frame,c,t.dist) #Find the area of the contour
					# if radius >= 50:
					if area >= 100:
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

						if (cX >= 480):
							cX = 479
						elif (cX <= 0):
							cX = 1


						color = fc.blurred[(cX,cY)]

						fc.colorcompare(color) #Find the closest matching color in the HSV colorspace

			cv2.imshow("Camera", t.frame)

		key = cv2.waitKey(1) & 0xFF
		# if the `q` key is pressed, break from the lop
		if key == ord("q"):
			print t.objectlist
			break