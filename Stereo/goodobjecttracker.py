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

		# self.yellowLower = (max(0, 20), 0, 0)
		# self.yellowUpper = (min(180, 50), 255, 255)
		self.yellowLower = (20, 100, 100)
		self.yellowUpper = (262, 274, 255)

		self.greenLower = (29,30,6)
		self.greenUpper = (64,255,255)

		self.Lower = [self.yellowLower,self.greenLower]
		self.Upper =[self.yellowUpper,self.greenUpper]

		self.objectlist = [[0,(0,0)]] # Radius, XY coordinates.

class Find_Color(object):
	def __init__(self, frame):
		self.blurred = cv2.GaussianBlur(frame, (21, 21), 0)
	# def findcolor(self):



if __name__=="__main__":

	t = Target()
	counter = 0

	while True:
		(grabbed, frame) = t.camera.read()
		# if the frame could not be grabbed, then we have reached the end
		# of the video

		# resize the frame, convert it to grayscale, and blur it
		t.frame = imutils.resize(frame, width=500)
		hsv = cv2.cvtColor(t.frame, cv2.COLOR_BGR2HSV)
		gray = cv2.cvtColor(t.frame, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (21, 21), 0)
		# if the first frame is None, initialize it
		if t.frame is None:
			t.frame = gray
			continue
		else:
			if t.count == 0:
				t.firstframe = gray
				fc = Find_Color(hsv)
				t.count += 1

			if t.thresh is not None:
				(cnts, _) = cv2.findContours(cv2.GaussianBlur(t.thresh.copy(), (25,25), 0), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
				# loop over the contours
				for c in cnts:
					# if the contour is too small, ignore it
					if cv2.contourArea(c) < t.args["min_area"]:
						continue
				# 	# compute the bounding box for the contour, draw it on the frame,
				# 	# and update the text
				# (x, y, w, h) = cv2.boundingRect(c)
				# cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
					(center,radius) = cv2.minEnclosingCircle(c)
					if radius >= 50:
						# print "radius: " , radius
						if center[0] >= 375:
							center = (374,center[1])

						cv2.circle(t.frame, (int(center[0]), int(center[1])), 5, (0,255,0), -1)
						cv2.circle(t.frame, (int(center[0]), int(center[1])), int(radius), (0,255,0), 5)
						# print "center", center
						color = fc.blurred[int(center[0]),int((center[1]))]

						# print radius

						if ((color[0] <= t.yellowUpper[0]) and (color[0] >= t.yellowLower[0])):
							# print 'Found yellow object'
							# print center
							# print color

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
									t.objectlist.append([radius,(0,0)])
									print 'Added object to list:' , [radius,(int(center[0]), int(center[1]))]
									print 'List is now: ' , t.objectlist
							counter +=1 

			cv2.imshow("Camera", t.frame)

		if t.firstframe is not None:
			frameDelta = cv2.absdiff(t.firstframe, gray)
			t.thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
			t.thresh = cv2.dilate(t.thresh, None, iterations=7)

			(cnts, _) = cv2.findContours(t.thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

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


# class Cluster(object):

#     def __init__(self):
#         self.pixels = []
#         self.centroid = None

#     def addPoint(self, pixel):
#         self.pixels.append(pixel)

#     def setNewCentroid(self):

#         R = [colour[0] for colour in self.pixels]
#         G = [colour[1] for colour in self.pixels]
#         B = [colour[2] for colour in self.pixels]

#         R = sum(R) / len(R)
#         G = sum(G) / len(G)
#         B = sum(B) / len(B)

#         self.centroid = (R, G, B)
#         self.pixels = []

#         return self.centroid


# class Kmeans(object):

#     def __init__(self, k=3, max_iterations=5, min_distance=5.0, size=200):
#         self.k = k
#         self.max_iterations = max_iterations
#         self.min_distance = min_distance
#         self.size = (size, size)

#     def run(self, image):
#         self.image = image
#         self.image.thumbnail(self.size)
#         self.pixels = numpy.array(image.getdata(), dtype=numpy.uint8)

#         self.clusters = [None for i in range(self.k)]
#         self.oldClusters = None

#         randomPixels = random.sample(self.pixels, self.k)

#         for idx in range(self.k):
#             self.clusters[idx] = Cluster()
#             self.clusters[idx].centroid = randomPixels[idx]

#         iterations = 0

#         while self.shouldExit(iterations) is False:

#             self.oldClusters = [cluster.centroid for cluster in self.clusters]

#             print iterations

#             for pixel in self.pixels:
#                 self.assignClusters(pixel)

#             for cluster in self.clusters:
#                 cluster.setNewCentroid()

#             iterations += 1

#         return [cluster.centroid for cluster in self.clusters]

#     def assignClusters(self, pixel):
#         shortest = float('Inf')
#         for cluster in self.clusters:
#             distance = self.calcDistance(cluster.centroid, pixel)
#             if distance < shortest:
#                 shortest = distance
#                 nearest = cluster

#         nearest.addPoint(pixel)

#     def calcDistance(self, a, b):

#         result = numpy.sqrt(sum((a - b) ** 2))
#         return result

#     def shouldExit(self, iterations):

#         if self.oldClusters is None:
#             return False

#         for idx in range(self.k):
#             dist = self.calcDistance(
#                 numpy.array(self.clusters[idx].centroid),
#                 numpy.array(self.oldClusters[idx])
#             )
#             if dist < self.min_distance:
#                 return True

#         if iterations <= self.max_iterations:
#             return False

#         return True