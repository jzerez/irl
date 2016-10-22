import cv2
import numpy as np

class BackgroundSubtractor:
    def __init__(self):
        self.fgbg = cv2.BackgroundSubtractorMOG2()
    def apply(self,image):
        return self.fgbg.apply(image, 0.5)
