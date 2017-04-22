#!/usr/bin/env python

# Imports
import roslib
import cv2
import rospy
import sys
import math
import time
import numpy as np
import random
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

"""ObjectInteraction class:
    A class that instantiates and runs the object interaction sequence.
    Edwin will first listen to determine the requested object.
    He will next convert the text command to search parameters.
    Then, he will use the search parameters to locate matching objects.
    Once the correct object is located, he will move to hover above it at a set height.

    This class contains state variables describing the game and functions to analyze 
    video data and perform robot operations.
    | see | | terminal output describing running processes
    | see | | Edwin runs a game of Push Cup
"""
class ObjectInteraction:
    def __init__(self, init=False):
        """__init__ function
        __init__ is a function run once when a new ObjectInteraction instance is
        created. The function initializes variables to store goal and object
        positions and other information. It also starts the publisher and
        subscriber nodes necessary to run Edwin, an SR-17 robotic arm.
        """

        print ("INIT| Initializing")
        # ---------- State Variables ----------

        self.debug = True # Debug State
        self.temp = 0
        self.cv_image = None

        # Positions
        self.obj_pos = [0,0]
        self.goal_pos = [0,0]

        # Dimensions
        self.screen_width = 640
        self.screen_height = 480
        self.gameboard_top = 6700
        self.gameboard_bottom = 3200
        self.gameboard_left = -2500
        self.gameboard_right = 3000

        # Motion Limits
        self.lowlimit_x = -1500
        self.highlimit_x = 2300
        self.lowlimit_y = 3200
        self.highlimit_y = 6700
        self.lowlimit_z = -1000
        self.highlimit_z = 4000

        # States
        self.timecounter = 0
        self.human_in_frame = False   # False if the Kinect does not detect human, True if he does
        self.seen_object = False     # False if Edwin has not located an object, True if he does
        self.locating_obj = True     # False if Edwin is not currently locating the object, True if he is.
        self.pickup_obj = False     # False if Edwin is not currently picking up an object, True if he is
        self.holding_obj = False     # False if Edwin is not currently holding an object, True if he is

        if not init:
            rospy.init_node('pickup_obj') # Creates node from which to subcribe and publish data

        # Sends data to Edwin
        self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=10)
        self.sim_pub = rospy.Publisher('/obj_pose', Pose, queue_size=10)

        # Creates Edwin's overview position route
        time.sleep(1)



        def startup(self):
            """startup
               startup is a function that runs when an instance of Pickup_Obj is first run. 
               Edwin moves to the "vulture" pose at 621, 4832, 3739, 845, 195, 0
            """


        # geometry_msgs::Pose msg1
        # dom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
        # msg1 = "position: x: 0.0  y: 0.0  z: 0.0 orientation:  x: 0.0  y: 0.0  z: 0.0  w: 0.0"

        #msg1 = "create_route:: R_vulture; 621, 4832, 3739, 845, 195, 0" # Positions multiplied by 10 from #s at top of code
        #print ("INIT| Sending: ", msg1
        #self.arm_pub.publish(msg1)
        # group.go(tpose,wait = True)

        # print ("INIT| Sending: ", self.pose.position.x, self.pose.position.y, self.pose.position.z, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)

        # self.sim_pub.publish(self.pose)
        # time.sleep(1)

        # Moves Edwin along route
        self.run_route("R_vulture")
        print ("INIT| Finished")

    # Runs once for every reciept of an image from usb_cam
    def callback(self, data):
        # print('hello!')

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converts usb cam feed to csv feed; bgr8 is an image encoding

            # Sets image size
            image_size = self.cv_image.shape
            screen_height = image_size[0]
            screen_width = image_size[1]

            cv2.imshow("Debug", self.cv_image)
            cv2.waitKey(0)

        except CvBridgeError as e:
            print(e)

    """ overview_pos function:
        Function: moves Edwin to a standardized position where he can examine the entire gameboard
        ----------------------------------------------------------------------------------
        |param | self | access to the state variables of the class calling the function  |
        |see   |      | Edwin moves to position                                          |
    """

    def make_pose(self, px, py, pz, ox, oy, oz, ow):
        """make_pose function. Used for creating a Pose message to publish to the arm. 
           The values and orientations specified in the input values define the orientation and location of the terminal end of the last link of an SR-17 robotic arm.
           
           makePose(X position, Y position, Z position, X orientation, Y orientation, Z orientation, W orientation)

           XYZ position is a Point, a 3D position located in real space.
           XYZW orientation a Quaternion, which describes how the final link is rotated in space. 
           Read more about quaternions here: http://answers.unity3d.com/questions/147712/what-is-affected-by-the-w-in-quaternionxyzw.html
        """

        self.pose = Pose()
        self.pose.position.x = px
        self.pose.position.y = py
        self.pose.position.z = pz
        self.pose.orientation.x = ox
        self.pose.orientation.y = oy
        self.pose.orientation.z = oz
        self.pose.orientation.w = ow

        print ("Sending Pose: ", px, py, pz, ox, oy, oz, ow)
        self.sim_pub.publish(self.pose)
        time.sleep(1)
        print ("Pose Sent.")
        return

    def run_route(self, route):
        msg2 = "run_route:: " + route
        print ("RUN| Sending: ", msg2)
        self.arm_pub.publish(msg2)
        time.sleep(2)

    # Manages contours (positions, areas, relevance, etc.)
    def apply_effects(self, feed):
        """apply_effects function. Used to visualize camera feeds for debugging and overlay effects on them.
            |param | self | access to the state variables of the class calling the function  |
        """

        blur = cv2.GaussianBlur(feed, (5,5), 0) # Gaussian Blur filter

        # Calls functions to contour cup and calculate moments
        contour, contours = self.contour_feed(blur)

        # Returns contoured feed only if 1+ contours present in image, else runs raw feed
        if len(contours) > 0:
            video = self.calculate(contour, contours)

            # Draws tracking dots
            cv2.circle(video,(self.obj_pos[0],self.obj_pos[1]),5,(0,0,255),-1)
            # cv2.circle(video,(self.goal_pos[0],self.goal_pos[1]),5,(0,0,0),-1)
        else:
            video = contour

        # print('hello!')

        # Feed Display(s) for debug:
        cv2.imshow('Raw Feed (feed)',feed)

        #cv2.imshow('Gaussian Blur Filter (blur)', blur)
        #cv2.imshow('Contour Filter (contour)', contour)

        # Final Contour feed
        cv2.imshow('Final Contours (video)', video)

        k = cv2.waitKey(5) & 0xFF

    # Contours video feed frame
    def contour_feed(self, video):

        contour = video # Duplicate video feed so as to display both raw footage and final contoured footage

        # Changes BGR video to GRAY and dynamically thresholds it [2]
        vidgray = cv2.cvtColor(video,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(vidgray,100,200,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        # Uses kernel to clean noise from image (2x) [1]
        kernel = np.ones((5, 5),np.uint8)
        opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)

        # Cleans out background through extra dilations (3x)
        sure_bg = cv2.dilate(opening,kernel,iterations=3)

        # Calculates contours
        contours, h = cv2.findContours(opening,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        # Creates list of contours with more points than 100 so as to select out for cup and hand
        finalcontours = [None]*2 # 1st Elem: Cup | 2nd Elem: Goal | 3rd Elem: Hand
        for cnt in contours:
            area_real = cv2.contourArea(cnt)
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            area_approx = math.pi*radius**2
            area_diff = area_approx - area_real
            diff_coefficient = area_diff / radius #Measure of how long and skinny something is. (Overall area covered / actual surface area.)
            #cv2.circle(contour,(int(x),int(y)),int(radius),(0,0,255),2) # Draws circles used for area comparison

            # If contour is really close to circle
            # if (diff_coefficient < 100) and (area_real > 200): #If obj is large enough and vaguely circular
            #     if (area_real < 600):                              #Check if object is smaller than 600.
            #         finalcontours[1] = cnt                         #If so, mark obj as "goal."
            #     elif (area_real > 5000):                           #Else, check if object is larger than 5000.
            #         finalcontours[0] = cnt                         #If so, mark obj as "cup"
            # if (area_real > 25000):                            #If obj is large enough, but not circular
            #         finalcontours[2] = cnt                         #Mark obj as "human."
            
            obj = self.obj_identify(finalcontours, diff_coefficient, area_real, cnt)

        #if self.debug == True:
            #cv2.drawContours(contour, contours, -1, (255,0,0), 3)
            #cv2.imshow('contour_feed: Final Video(contour)',contour)

        if self.seen_object == True: cv2.drawContours(contour, finalcontours[0], -1, (0,0,255), 3)
        cv2.circle(contour, (self.screen_width/2, self.screen_height/2), 5, (0,0,255), -1)
        # if self.goal_in_frame == True: cv2.drawContours(contour, finalcontours[1], -1, (0,0,0),3)
        if self.human_in_frame == True: cv2.drawContours(contour, finalcontours[1], -1, (0,255,0),3)

        # Feed Display(s) for debug:
        #cv2.imshow('contour_feed: Raw Video(video)',video)
        #cv2.imshow('contour_feed: To GRAY Filter (vidgray)',vidgray)
        #cv2.imshow('contour_feed: Threshold Filter (thresh)',thresh)
        #cv2.imshow('contour_feed: Opening Kernel (opening)', opening)
        #cv2.imshow('contour_feed: Background Clear (sure_bg)', sure_bg)

        return contour, finalcontours

    def obj_identify(self, fincnt, diff_coeff, area_real, cnt):
        fincnt[0] = None
        fincnt[1] = None

        if area_real > 500:
            print("Obj size: ", area_real)

            if ((diff_coeff < 100) and (area_real > 5000)): #If obj is large enough and vaguely circular
                fincnt[0] = cnt                             #If so, mark obj as "cup"
                print("Object found.")

            if ((diff_coeff >= 100) and (area_real > 25000)): #If obj is large enough, but not circular
                fincnt[1] = cnt                               #Mark obj as "human."
                print("Human found.")
        else:
            pass

        return fincnt

    # Center & Movement Detection Function
    def calculate(self, contour, finalcontours):

        video = contour

        #--------------------Unpacks finalcontours--------------------#

        # Object contour
        if (finalcontours[0] != None):
            obj_contour = finalcontours[0]
            self.seen_object = True
            obj_moments = cv2.moments(obj_contour)

            # Calculates xy values of centroid
            if obj_moments['m00']!=0:
                self.obj_pos[0] = int(obj_moments['m10']/obj_moments['m00'])
                self.obj_pos[1] = int(obj_moments['m01']/obj_moments['m00'])
        else:
            self.seen_object = False

        # Hand contour
        if (finalcontours[1] != None):
            hand_contour = finalcontours[1]
            self.hand_in_frame = True
        else:
            self.hand_in_frame = False

        # Feed Display(s) for debug:
        #cv2.imshow('calculate: Raw Video(contour)',contour)
        #cv2.imshow('calculate: Centroid Draw(video)',video)

        return video

    """ check_pos function:
        check_pos ensures that moving Edwin to a position doesn't breach the limits placed on his movement
    """
    def check_pos(self,pos):
        safe = False
        if (self.lowlimit_x < pos[0] < self.highlimit_x):
            if (self.lowlimit_y < pos[1] < self.highlimit_y):
                if (self.lowlimit_z < pos[2] < self.highlimit_z):
                    safe = True
        return safe

    def convert_space(self, pos):
        """ convert_space function
        convert_space converts an xy point in camspace (pixel location) to
        realspace (edwin head location)
        ------------------------------------------------------------------------
        |param  | self | access to the state variables of the class calling    |
        |the function                                                          |
        |param  | x    | x position of point in camspace                       |
        |param  | y    | y position of point in camspace                       |
        |return |      | vector of cup x and y position in realspace           |
        """

        if self.debug == True: print("SPC: Old Coordinates: ", pos[0], pos[1])

        # Determines equations to convert from camspace to realspace in x-direction
        m1 = (2000 - 300)/(484 - 305)
        b1 = - 2000
        x_real = m1 * pos[0] + b1

        # Determines equations to convert from camspace to realspace in Y-direction
        m2 = (6900 - 4900)/(127 - 352)
        b2 = 7500
        y_real = m2 * pos[1] + b2

        if self.debug == True: print("SPC: New Coordinates: ", x_real, y_real)
        return([x_real, y_real])

    def locate_object(self, color, vid):
        """ locate_object function
        locate_object controls Edwin once he has been directed to point at a certain object, until he is at the set position above the object.
        ---------------------------------------------------------------------------------
        |param | self | access to the state variables of the class calling the function |

        """

        if vid == None:
            print("No video feed. \n")
            return

        else:
            if self.debug == True: print("DBG| Looking for object!")

            self.apply_effects(self.cv_image) # Image processing
            time.sleep(2)

            if self.human_in_frame:
                pass
            else:
                if self.seen_object == True: # If game pieces are on the playing field

                    if self.debug == True: print ("Located Object!\nMoving to object location.")
                    self.locating_obj = False
                    self.pickup_obj = True
                    return

                else: # If some game pieces are missing
                    if self.debug == True:
                        if self.seen_object == False: print("ERROR: No cup in frame")
                        # if self.goal_in_frame == False: print("ERROR: No goal in frame")

    def move_to_obj(self):
        """ move_to_obj function:
        move_to_obj makes Edwin move to hover a set distance above the specified object found by locate_object.
        Distance yet to be determined.
        ---------------------------------------------------------------------------------
        |param | self | access to the state variables of the class calling the function |
        |see   |      | Edwin pushes or pulls the cup such that it covers the goal      |
        """

        # self.sim_pub.publish( "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0")
        print("CLB| Obj Pos Screenspace: ", self.obj_pos)
        print("CLB| Obj Pos Realspace: ", self.convert_space(self.obj_pos))
        time.sleep(1)

        # print("hi! :)")

        # # Converts game piece positions into realspace
        # cup = self.convert_space(self.cup_pos)
        # goal = self.convert_space(self.goal_pos)
        # if self.debug == True: # Debugging info
        #     print ("PSH| pos = ", cup)
        #     print ("PSH| goal pos = ", goal)

        # # Checks if positions are safe [TOFIX]
        # if (self.check_pos([cup[0], cup[1], -500]) == True):

        # #---------------ROUTE CONSTRUCTION---------------#
        #     # Determines direction to push cup
        #     direction = None
        #     msg1 = "create_route:: R_push; "
        #     if cup[0] < goal[0]: direction = "Right"
        #     if cup[0] > goal[0]: direction = "Left"

        #     if cup[1] < goal[1]: # Position Below Cup; Push Up to Goal Y
        #         print("PSH| Pushing Up")
        #         msg1 = msg1 + str(cup[0]) + ", " + str(cup[1] - 300) + ", -800, 845, 195, 0"                    # Head Down
        #         msg1 = msg1 + ", " + str(cup[0]) + ", " + str(goal[1]) + ", -800, 845, 195, 0"                  # Push Forward
        #         msg1 = msg1 + ", " + str(cup[0]) + ", " + str(goal[1] - 100) + ", -800, 845, 195, 0"            # Disengage

        #         if direction == "Left": # Position Right of Cup; Push Left to Goal X
        #             print("PSH| Pushing Left")
        #             msg1 = msg1 + ", " + str(cup[0] + 800) + ", " + str(goal[1] + 800) + ", -800, 845, 195, 0"        # Move Right
        #             msg1 = msg1 + ", " + str(cup[0] + 800) + ", " + str(goal[1]) + ", -800, 845, 195, 0"  # Move Up
        #             msg1 = msg1 + ", " + str(goal[0] + 300) + ", " + str(goal[1]) + ", -800, 845, 195, 0"       # Push Left
        #         elif direction == "Right": # Position Left of Cup; Push Right to Goal Y
        #             print("PSH| Pushing Right")
        #             msg1 = msg1 + ", " + str(cup[0] - 300) + ", " + str(goal[1]) + ", -800, 845, 195, 0"        # Move Left
        #             msg1 = msg1 + ", " + str(cup[0] - 300) + ", " + str(goal[1] + 300) + ", -800, 845, 195, 0"  # Move Up
        #             msg1 = msg1 + ", " + str(goal[0]) + ", " + str(goal[1] + 300) + ", -800, 845, 195, 0"       # Push Right
        #     else: # Position Above Cup; Push Down to Goal Y
        #         print("PSH| Pushing Down")
        #         msg1 = msg1 + str(cup[0]) + ", " + str(goal[1] + 1000) + ", -800, 845, 195, 0"           # Push Backward
        #         msg1 = msg1 + ", " + str(cup[0]) + ", " + str(goal[1] + 1100) + ", -800, 845, 195, 0"           # Disengage

        #         if direction == "Left": # Position Right of Cup; Push Left to Goal X
        #             print("PSH| Pushing Left")
        #             msg1 = msg1 + ", " + str(cup[0] + 800) + ", " + str(goal[1] + 1100) + ", -800, 845, 195, 0" # Move Right
        #             msg1 = msg1 + ", " + str(cup[0] + 800) + ", " + str(goal[1]) + ", -800, 845, 195, 0"        # Move Down
        #             msg1 = msg1 + ", " + str(goal[0]) + ", " + str(goal[1]) + ", -800, 845, 195, 0"             # Push Left
        #         elif direction == "Right": # Position Left of Cup; Push Right to Goal Y
        #             print("RSH| Pushing Right")
        #             msg1 = msg1 + ", " + str(cup[0] - 800) + ", " + str(goal[1] + 1100) + ", -800, 845, 195, 0"       # Move Left
        #             msg1 = msg1 + ", " + str(cup[0] - 800) + ", " + str(goal[1]) + ", -800, 845, 195, 0" # Move Down
        #             msg1 = msg1 + ", " + str(goal[0]) + ", " + str(goal[1]) + ", -800, 845, 195, 0"      # Push Right

        #     # Creates and runs route, then returns to overview
        #     print ("PSH| Sending: ", msg1)
        #     self.arm_pub.publish(msg1)
        #     time.sleep(1)
        #     self.run_route("R_push")
        #     self.run_route("R_vulture")

    def run(self):
        """ run function:
        run handles the actual updating of the code and overall state of the program.
        ---------------------------------------------------------------------------------
        |param | self | access to the state variables of the class calling the function |
        """

        r = rospy.Rate(20) # Sets update rate
        # Gets data from usb_cam
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/my_stereo/left/image_color",Image,self.callback) # Determines node for subscription

        color = (0,0,255) # Temporary marker for color
        # Insert voice to text code here
        # Insert text to color code here

        while not rospy.is_shutdown():
            r.sleep()
            # if self.debug == True: self.calibrate()
            if self.locating_obj == True:
                self.locate_object(color, self.cv_image)
            if self.pickup_obj == True:
                self.move_to_obj()


if __name__=='__main__':
    print "in main"
    oi = ObjectInteraction() # Creates new instance of class ObjectInteraction
    oi.run() # Calls run function
    cv2.destroyAllWindows()