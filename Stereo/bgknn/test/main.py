import numpy as np
import cv2
import libopencv_bgknn

cap = cv2.VideoCapture(0)

fgbg = libopencv_bgknn.BackgroundSubtractorKNN(400,500,True)

while True:
    _,frame = cap.read()
    mask = np.zeros(frame.shape)
    mask = fgbg.apply(frame, 0.1)
    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    if cv2.waitKey(200) == 27:
        break
