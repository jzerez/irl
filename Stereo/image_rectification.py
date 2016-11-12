import cv2

cam_l = cv2.VideoCapture(0)
cam_r = cv2.VideoCapture(1)

# TODO : load m_l, d_l, r_l, p_l,
# and m_r, d_r, r_r, and p_r

c_l_m1, c_l_m2 = cv2.initUndistortRectifyMap(m_l,d_l,r_l,p_l, (640,480), cv2.CV_8SC3)
c_r_m1, c_r_m2 = cv2.initUndistortRectifyMap(m_r,d_r,r_r,p_r, (640,480), cv2.CV_8SC3)

# TODO : execute following code every time capture occurs ...
while True:
    ret_l, left = cam_l.read()
    ret_r, right = cam_r.read()
    
    left = cv2.remap(left, c_l_m1, c_l_m2, cv2.INTER_LINEAR)
    right = cv2.remap(right, c_r_m1, c_r_m2, cv2.INTER_LINEAR)

    cv2.imshow("left", left)
    cv2.imshow("right", right)

