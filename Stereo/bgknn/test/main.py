import numpy as np
import cv2
import sys, os

path = os.path.join(os.path.dirname(__file__) + '../build/')
sys.path.append(path)

from libopencv_bgknn import BackgroundSubtractorKNN

cap = cv2.VideoCapture(0)

fgbg = BackgroundSubtractorKNN(400,500,True)

while True:
    _,frame = cap.read()
    mask = np.zeros(frame.shape)
    mask = fgbg.apply(frame, 0.1)
    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    if cv2.waitKey(20) == 27:
        break
