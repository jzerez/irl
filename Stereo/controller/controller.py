#!/usr/bin/python
import numpy as np
import rospy

from std_msgs.msg import Float32
from geometry_msgs.msg import Point

class PID(object):
    def __init__(self, k_p, k_i, k_d):
        self.k_p, self.k_i, self.k_d = k_p, k_i, k_d
        self.res = 0.0
        self.e_i = self.e_d = 0.0
        self.error = 0.0
    def compute(self, err, dt):
        if dt == 0:
            return self.res
        k_p, k_i, k_d = self.k_p, self.k_i, self.k_d
        self.e_i += err*dt
        self.e_d = self.error - err 
        self.res = k_p*err + k_i*self.e_i*dt + k_d*self.e_d/dt
        self.error = err
        return self.res

def angle_err(x, fov):
    theta = fov/2
    # angle difference
    a = x / 640. # assuming 640 is frame width
    return np.arctan2(2*a*np.sin(theta)-np.sin(theta), np.cos(theta))

x = 320 # middle
y = 240 # middle

def get_pos(msg):
    global x, y
    x = msg.x
    y = msg.y

def main():
    global x,y
    fov = np.deg2rad(65)
    pid = PID(1.0,0.0,0.0)

    rospy.init_node('demo')
    rate = rospy.Rate(100.0)

    pub = rospy.Publisher('move_dir',Float32,queue_size=10) 
    sub = rospy.Subscriber('obj_pos',Point, get_pos)

    then = rospy.get_time()
    while not rospy.is_shutdown():
        err = angle_err(x, fov)
        now = rospy.get_time()
        dt = (now - then)
        then = now

        val = pid.compute(err, dt) 
        print err,val

        pub.publish(val)
        rate.sleep()

if __name__ == "__main__":
    main()
