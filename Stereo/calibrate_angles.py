import numpy as np
import cv2

def mouse_cb(ev,x,y,flag,data):
    if flag == 1:
        print (x,y)

if __name__ == "__main__":
    cam = cv2.VideoCapture(0)
    cv2.namedWindow('img', 1)
    cv2.setMouseCallback('img', mouse_cb)
    while True:
        _, img = cam.read()
        cv2.imshow('img', img)
        if cv2.waitKey(10) == 27:
            break

