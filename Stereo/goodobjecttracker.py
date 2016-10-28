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


if __name__=="__main__":

	t = Target()
	count = 0
	firstframes = []
	thresh = None
	firstframe = None

	while True:
		(grabbed, frame) = t.camera.read()
		# if the frame could not be grabbed, then we have reached the end
		# of the video

		# resize the frame, convert it to grayscale, and blur it
		frame = imutils.resize(frame, width=500)
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (21, 21), 0)
		# if the first frame is None, initialize it
		if frame is None:
			frame = gray
			continue
		else:
			if count == 0:
				firstframe = gray
				count += 1

			if thresh is not None:
				(cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
				# loop over the contours
				for c in cnts:
					# if the contour is too small, ignore it
					if cv2.contourArea(c) < t.args["min_area"]:
						continue
				# 	# compute the bounding box for the contour, draw it on the frame,
				# 	# and update the text
					(center,radius) = cv2.minEnclosingCircle(c)
					cv2.circle(frame, (int(center[0]), int(center[1])), 5, (0,255,0), -1)
					cv2.circle(frame, (int(center[0]), int(center[1])), 100, (0,255,0), 5)
					# (x, y, w, h) = cv2.boundingRect(c)
					# cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

			cv2.imshow("Camera", frame)

		if firstframe is not None:
			frameDelta = cv2.absdiff(firstframe, gray)
			thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
			thresh = cv2.dilate(thresh, None, iterations=7)

			(cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

			cv2.imshow("Static Frame", thresh)

		if np.average(thresh) > 200:
			firstframe = gray

		key = cv2.waitKey(1) & 0xFF
		# if the `q` key is pressed, break from the lop
		if key == ord("q"):
			break
		if key == ord("r"):
			firstframe = gray

	# compute the absolute difference between the current frame and
	# # first frame
	# frameDelta = cv2.absdiff(self.firstframe, gray)
	# thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

	# # dilate the thresholded image to fill in holes, then find contours
	# # on thresholded image
	# thresh = cv2.dilate(thresh, None, iterations=2)
	# (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
	# 	cv2.CHAIN_APPROX_SIMPLE)

	# # loop over the contours
	# for c in cnts:
	# 	# if the contour is too small, ignore it
	# 	if cv2.contourArea(c) < args["min_area"]:
	# 		continue

	# 	# compute the bounding box for the contour, draw it on the frame,
	# 	# and update the text
	# 	(x, y, w, h) = cv2.boundingRect(c)
	# 	cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
	# 	text = "Occupied"

	# # draw the text and timestamp on the frame
	# cv2.putText(frame, "Room Status: {}".format(text), (10, 20),
	# 	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
	# cv2.putText(frame, datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"),
	# 	(10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

	# # show the frame and record if the user presses a key
	# cv2.imshow("Security Feed", frame)
	# cv2.imshow("Thresh", thresh)
	# cv2.imshow("Frame Delta", frameDelta)