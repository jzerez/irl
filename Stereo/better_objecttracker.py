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
import rospy
from geometry_msgs import Point
from background_subtraction import BackgroundSubtractor

bksub = BackgroundSubtractor()

class Target(object):
	def __init__(self, passed_umod_frame, passed_dist, passed_bkg_mask):
		self.input = "small blue object"

		self.frame = passed_umod_frame
		(rows,cols,color) = dim.shape
		self.dim = [rows,cols]
		self.dist = passed_dist

		#Camera FoV info
		self.xFoV = 32.5 #Degrees, the field of vision on the x axis
		self.yFoV = 20.9 #Degrees

		# construct the argument parser and parse the arguments
		ap = argparse.ArgumentParser()
		
		ap.add_argument("-a", "--min-area", type=int, default=1500, help="minimum area size")
		self.args = vars(ap.parse_args())

		self.count = 0
		self.bkg_mask = passed_bkg_mask

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

		self.objectlist = []#[[0,(0,0),'Null']] # Contour area, XY coordinates.

class Process_Info(object):
	def __init__(self, passed_command):
		cmdstr = passed_command

		colors = ['blue', 'red', 'green']
		sizes = ['small', 'medium', 'large']

		cmdwrds = cmdstr.split()
		self.findcommands(colors, sizes, cmdwrds)

	def findcommands(self, passed_colors, passed_sizes, passed_cmdwrds)
		for word in cmdwrds:
			for color in colors:
				if word.lower() == color.lower():
					self.color = color
			for size in sizes:
				if word.lower() == size.lower():
					self.size = size
		print "Looking for a %s %s thing!".format{self.size, self.color}

	def newcommand(self, passed_command):
		cmdstr = passed_command

		colors = ['blue', 'red', 'green']
		sizes = ['small', 'medium', 'large']

		cmdwrds = cmdstr.split()

		self.objcolor = "Null"
		self.objsize = "Null"
		self.finalpubval = [] #[x,y,dist]

		self.findcommands(colors, sizes, cmdwrds)


class Locate_Object(object):
	def __init__(self, frame):
		self.blurred = cv2.GaussianBlur(frame, (21, 21), 0)
		self.objcolor = "Null"
		self.objsize = "Null"
		self.finalpubval = [] #[x,y,dist]

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
			t.objectlist.append([area,(cX, cY),lo.objcolor])
			print 'Added object to list:' , [area,(int(center[0]), int(center[1])),str(lo.objcolor)]
			print lo.objcolor
			print 'List is now: ' , t.objectlist

	def areacalc(self, contour, passed_dist):
		[xdim,ydim] = t.dim
		dist = passed_dist

		x,y,w,h = cv2.boundingRect(contour)
		centerx = x+(0.5*w)
		centery = y+(0.5*h)

		xmin_theta = x * (t.xFoV/xdim)
		ymin_theta = y * (t.yFoV/ydim)
		xmax_theta = (x+w) * (t.xFoV/xdim)
		ymax_theta = (y+h) * (t.yFoV/ydim)

		xmin_width = np.tan(xmin_theta) * dist
		ymin_length = np.tan(ymin_theta) * dist
		xmax_width = np.tan(xmax_theta) * dist
		ymax_length = np.tan(ymax_theta) * dist

		xwidth = xmax_length - xmin_length
		ylength = ymax_length - ymin_length

		area = xwidth * ylength
		coords = [x,y]
		return [area,coords]

	def colorcompare(self, color):
		if (((color[0] <= t.redUpper[0]) and (color[0] >= t.redLower[0])) or ((color[0] <= t.redUpper1[0]) and (color[0] >= t.redLower1[0]))):
			lo.objcolor = "Red"
			lo.findobject()
		if ((color[0] <= t.greenUpper[0]) and (color[0] >= t.greenLower[0])):
			lo.objcolor = "Green"
			lo.findobject()
		if ((color[0] <= t.blueUpper[0]) and (color[0] >= t.blueLower[0])):
			lo.objcolor = "Blue"
			lo.findobject()

	def passcoords(self, passed_color, passed_area, passed_coords, passed_dist):
		objcolor = passed_color
		area = passed_area
		coords = passed_coords
		dist = passed_dist

		if (area <= 9.0): #inches
			lo.objsize = "Small"
		else if ((area > 9.0) && (area <= 25.0)):
			lo.objsize = "Medium"
		else if (area > 25.0):
			lo.objsize = "Large"

		if (objcolor.lower() == pi.color.lower()): #If the object color matches the target color...
			if (objsize.lower() == pi.size.lower()): #If the object size matches the target size...
				finalpubval = [coords,dist]

	# def objectpoint(self, theta, dist):
	# 	area = areacalc(tracked_cnt,dist)
	# 	[xmin_theta,xmax_theta,ymin_theta,ymax_theta] = theta
	# 	pt_angle = (xmax_theta+xmintheta) / 2

	# 	objspd = 1.0
	# 	return [pt_angle, obj_spd]




if __name__=="__main__":

	command = "small blue object"

	pi = Process_Info(command)
	t = Target(umod_frame, dist, bkg_mask)

	while True:
		
		hsv = cv2.cvtColor(t.frame, cv2.COLOR_BGR2HSV)
		lo = Locate_Object(hsv)
		
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
					[area, coords] = lo.areacalc(t.frame,c,t.dist) #Find the area of the contour
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
						
						cv2.imshow('blr', lo.blurred)

						# color = lo.blurred[int(center[1]),int((center[0]))]

						if (cX >= 480):
							cX = 479
						elif (cX <= 0):
							cX = 1


						color = lo.blurred[(cX,cY)]

						lo.colorcompare(color) #Find the closest matching color in the HSV colorspace
						lo.passcoords(lo.objcolor, area, coords, t.dist) #Check to see if object stats match object being tracked, then transmit obj. coords

						# [pt_angle, obj_spd] = lo.objectpoint(thetalist,tracked_cnt,t.dist)

			pub = rospy.Publisher('obj_pos', Point, queue_size=10)

			publish valuesssssssss
			cv2.imshow("Camera", t.frame)

		key = cv2.waitKey(1) & 0xFF
		# if the `q` key is pressed, break from the lop
		if key == ord("q"):
			print t.objectlist
			break