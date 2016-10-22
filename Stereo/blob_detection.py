import cv2
import numpy as np

def circleArea(r):
    return r*r*3.14159265358979

class BlobDetector:
    def __init__(self):
        #BLOB DETECTION ...
        params = cv2.SimpleBlobDetector_Params()
        params.minDistBetweenBlobs = 0

        params.filterByColor = False 
        params.blobColor = 255

        params.filterByArea = True 
        params.minArea = 400.
        params.maxArea = 999999999.

        params.filterByCircularity = False

        params.filterByConvexity = True 
        params.minConvexity = 0.5

        params.filterByInertia = False

        detector = cv2.SimpleBlobDetector(params)
        self.detector = cv2.SimpleBlobDetector(params)

    def apply(self, img):
        identified = img.copy()
        #identified = np.zeros(image.shape, dtype=image.dtype)
        k_dilate = np.asarray([
            [.07,.12,.07],
            [.12,.24,.12],
            [.07,.12,.07]
            ],np.float32)

        proc = cv2.dilate(img, k_dilate, iterations = 5) # fill the holes
        labels = self.detector.detect(proc)
        identified = cv2.drawKeypoints(proc,labels,flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return len(labels), identified
