#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from pynput import keyboard
from pynput.keyboard import Key, Listener
import random
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import imutils


class project1():

    def __init__(self):

        #this twist message is used throughout the class whenever we want to send movement messages to the robot
        self.move_cmd = Twist()

        # used for opencv object detection
        self.bridge_object = CvBridge()

        #various flags used to determine whether or not we have certain types of input that wil pre-empt others
        self.isBumped = False
        self.keyPressed = False
        self.isSymmetric = False
        self.isAsymmetricLeft = False
        self.isAsymmetricRight = False
        self.obstacleInSight = False
        self.landmarkDrive = False
        self.landmarkLeft = False
        self.landmarkRight = False

        #robot navigation messages will be published to this topic
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        #initiation the main project node that our code runs from
        rospy.init_node('project1')

        #we subscribe to these topics in order to determine when the robot bumps into an object and to get its Laser Scan and Image data
        self.bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.BumperCallback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.LaserCallback)
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_callback)

        #on shutdown (when Ctrl + C is pressed) we execute our custom shutdown function
        rospy.on_shutdown(self.shutdown)

        #this listener checks for keyboard input and refers it to our custom functions
        self.listener = keyboard.Listener(
            on_press = self.on_press,
            on_release = self.on_release)
        self.listener.start()
        
        #we define the rate that the robot checks for instructions as 10 Hz
        self.r = rospy.Rate(10)
        
        #this is the main while loop - code will continuously loop here and execute every tick until the node is shut down
        while not rospy.is_shutdown():
            #if the bumpers are engaged, stop the robot by publishing an empty Twist message (has linear and angular velocity zero)
            if(self.isBumped):
                self.cmd_vel.publish(Twist())
            #if the bumpers are NOT engaged, check for teleoperation
            elif(self.keyPressed):
                #listener above handles all key input, so this just makes sure that no other Twist commands are sent
                continue
            #if bumpers are NOT engaged and no arrow keys are pressed, engage normal movement
            else:
                #if symmetric obstacle is detected, complete full 180 spin and then continue
                if(self.isSymmetric):
                    #rospy.loginfo(self.obstacleInSight)
                    #setting our desired angular velocity and desired angle
                    speed = 75
                    angle = 180

                    #Converting from degrees to radians
                    angular_speed = speed*2*math.pi/360
                    relative_angle = angle*2*math.pi/360

                    #set all other velocities to zero
                    self.move_cmd.linear.x=0
                    self.move_cmd.linear.y=0
                    self.move_cmd.linear.z=0
                    self.move_cmd.angular.x=0
                    self.move_cmd.angular.y=0

                    #set rotation to our angular velocity
                    self.move_cmd.angular.z=angular_speed

                    #set the current time for distance calculus
                    t0 = rospy.Time.now().to_sec()
                    current_angle = 0

                    #loop until the entire angle has been traversed, publishing each loop
                    while (current_angle < relative_angle):
                        self.cmd_vel.publish(self.move_cmd)
                        t1 = rospy.Time.now().to_sec()
                        current_angle = angular_speed*(t1-t0)

                    #stop the robot by publishing an empty Twist message (has linear and angular velocity zero)
                    self.cmd_vel.publish(Twist())
                                    
                            
                #if asymmetric obstacle is detected on the left, veer right
                elif(self.isAsymmetricLeft):
                    # rospy.loginfo(self.obstacleInSight)
                    #small angle of rotation with high speed so that we can veer rather than turning
                    speed = 30
                    angle = 10

                    #Converting from degrees to radians
                    angular_speed = speed*2*math.pi/360
                    relative_angle = angle*2*math.pi/360

                    #set all other velocities to zero
                    self.move_cmd.linear.x=0
                    self.move_cmd.linear.y=0
                    self.move_cmd.linear.z=0
                    self.move_cmd.angular.x=0
                    self.move_cmd.angular.y=0

                    #set our angular velocity
                    self.move_cmd.angular.z= -(angular_speed)

                    #set the current time for distance calculus
                    t0 = rospy.Time.now().to_sec()
                    current_angle = 0

                    #loop until the entire angle has been traversed, publishing each loop
                    while (current_angle < relative_angle):
                        self.cmd_vel.publish(self.move_cmd)
                        t1 = rospy.Time.now().to_sec()
                        current_angle = angular_speed*(t1-t0)

                    #stop the robot by publishing an empty Twist message (has linear and angular velocity zero)
                    self.cmd_vel.publish(Twist())


                #if asymmetric obstacle is detected on the right, veer left
                elif(self.isAsymmetricRight):
                    #small angle of rotation with high speed so that we can veer rather than turning
                    speed = 30
                    angle = 10

                    #Converting from degrees to radians
                    angular_speed = speed*2*math.pi/360
                    relative_angle = angle*2*math.pi/360

                    #set all other velocities to zero
                    self.move_cmd.linear.x=0
                    self.move_cmd.linear.y=0
                    self.move_cmd.linear.z=0
                    self.move_cmd.angular.x=0
                    self.move_cmd.angular.y=0

                    #set our angular velocity
                    self.move_cmd.angular.z= angular_speed

                    #setting the current time for distance calculus
                    t0 = rospy.Time.now().to_sec()
                    current_angle = 0
                    #loop until the entire angle has been traversed, publishing each loop
                    while (current_angle < relative_angle):
                        self.cmd_vel.publish(self.move_cmd)
                        t1 = rospy.Time.now().to_sec()
                        current_angle = angular_speed*(t1-t0)

                    #stop the robot by publishing an empty Twist message (has linear and angular velocity zero)
                    self.cmd_vel.publish(Twist())
                #if there are no obstacles detected, see if we've found a landmark, turn as needed
                elif self.landmarkLeft:
                    # these movement commands equal a slow left turn
                    self.move_cmd.linear.x=0
                    self.move_cmd.linear.y=0
                    self.move_cmd.linear.z=0
                    self.move_cmd.angular.x=0
                    self.move_cmd.angular.y=0
                    self.move_cmd.angular.z = 0.05
                    self.cmd_vel.publish(self.move_cmd)
                elif self.landmarkRight:
                    # these movement commands equal a slow right turn
                    self.move_cmd.linear.x=0
                    self.move_cmd.linear.y=0
                    self.move_cmd.linear.z=0
                    self.move_cmd.angular.x=0
                    self.move_cmd.angular.y=0
                    self.move_cmd.angular.z = -0.05
                    self.cmd_vel.publish(self.move_cmd)
                elif self.landmarkDrive and not self.isSymmetric:
                    # drive forwards
                    self.move_cmd.linear.x=0.5
                    self.move_cmd.linear.y=0
                    self.move_cmd.linear.z=0
                    self.move_cmd.angular.x=0
                    self.move_cmd.angular.y=0
                    self.move_cmd.angular.z=0
                    self.cmd_vel.publish(self.move_cmd)
                # if there are no obstacles detected, drive forwards and turn every meter
                elif not(self.obstacleInSight):
                    #drive one meter, stopping if any other events occur
                    self.drive(0.3, 0.3)
                    #turn randomly, stopping if any bump or key press events occur
                    random_turn = random.uniform(-15, 15) #in degrees
                    self.turn(random_turn, 5)
                    
        #wait until next tick, makes sure that instructions are sent synchronously        
        self.r.sleep()
                        
    #stops turtlebot and sleeps until process is ended    
    def shutdown(self):
        rospy.loginfo("Stop TurtleBot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    #used to perform the random 15 degree turns the robot completes
    def turn(self, angle, speed):

        #exit out of the program if either of the two higher-priority actions take place
        if(self.isBumped or self.keyPressed):
            return

        #create a twist to output
        velocity_msg = Twist()

        #Converting from angles to radians
        angular_speed = speed*2*math.pi/360
        relative_angle = angle*2*math.pi/360

        #set angular speed
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = angular_speed
        
        #set the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        
        #loop until the entire angle has been traversed, publishing each loop
        while(current_angle < relative_angle):
            #exit out of the program if either of the two higher-priority actions take place
            if(self.isBumped or self.keyPressed):
                return
            self.cmd_vel.publish(velocity_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)
            
        #stop the robot by publishing an empty Twist message (has linear and angular velocity zero)
        self.cmd_vel.publish(Twist())

    #makes the robot go forwards a specified distance at a given speed
    def drive(self, distance, speed):

        #exit the program if any of the higher-priority actions occur
        if(self.isBumped or self.keyPressed or self.isSymmetric or self.isAsymmetricLeft or self.isAsymmetricRight):
            return

        #set our linear velocity
        velocity_msg = Twist()
        velocity_msg.angular.z = 0
        velocity_msg.linear.x = speed
        
        #set the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        
        #loop until the entire angle has been traversed, publishing each loop
        while(current_distance < distance):
            #exit the program if any of the higher-priority actions occur
            if(self.isBumped or self.keyPressed or self.isSymmetric or self.isAsymmetricLeft or self.isAsymmetricRight or self.obstacleInSight):
                return
            self.cmd_vel.publish(velocity_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = speed*(t1-t0)
            
        #stop the robot by publishing an empty Twist message (has linear and angular velocity zero)
        self.cmd_vel.publish(Twist())

    #used to capture bumper event data
    def BumperCallback(self, data):
        #if bumper is pressed, set the boolean flag to True (used in our init to determine precedence)
        if (data.state == 1):
            self.isBumped = True
        #if bumper is not pressed, set flag to false
        elif (data.state == 0):
            self.isBumped = False
    
    #640 values from our laser
    #middle 108(straight ahead and +- about 15 degrees) are used for symmetric detection
    #rest are used for automatic avoidance and escaping
    def LaserCallback(self, data):
        #set values as false, only changing them if there is a symmetric or asymmetric obstacle within 1 foot
        self.isSymmetric = False
        self.isAsymmetricLeft = False
        self.isAsymmetricRight = False
        self.obstacleInSight = False
        #symmetric detection
        for i in range(266, 374): #middle 108 values (about 30 degrees)
            #if any values in that range are less than 1/2 meter (the laser is ineffective at a foot, this is as close as we could get),
            #set the flag to true and exit the loop
            if(data.ranges[i] <= 0.6):
                self.isSymmetric = True
                self.obstacleInSight = True
                break
            else:
                #if there are no obstacles directly in front of the robot within that distance, set the flag to False
                self.isSymmetric = False
                self.obstacleInSight = False
        #asymmetric detection
        for i in range(375, 639): #scans on the left side of the robot
            #same logic as the symmetric detection, see above for an explanation
            if(data.ranges[i] <= 0.5):
                self.isAsymmetricLeft = True
                self.obstacleInSight = True
                break
            else:
                #if there are no obstacles on the left of the robot within that distance, set the flag to False
                self.isAsymmetricLeft = False
                self.obstacleInSight = False
        for i in range(0, 265):
            #same logic as the symmetric detection, see above for an explanation
            if(data.ranges[i] <= 0.5):
                self.isAsymmetricRight = True
                self.obstacleInSight = True
                break
            else:
                #if there are no obstacles on the right of the robot within that distance, set the flag to False
                self.isAsymmetricRight = False
                self.obstacleInSight = False

    #sets various flags based on whether or not an object is detected
    def camera_callback(self,data):
        #get image from camera
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        #get image bounds
        height, width, channels = cv_image.shape

        #Convert from BGR to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Determine the lower and upper HSV color values to track, this helps
        # with color variants depending on the lighting in the environment
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([30,255,255])

        # Apply mask
        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # find contours in the thresholded image
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,
	        cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        for c in cnts:
	        # compute the center of the contour
            M = cv2.moments(c)
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            #draw contours as green lines
            cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
            #draw centroid as white circle
            cv2.circle(cv_image, (cx, cy), 7, (255, 255, 255), -1)

            # if centroid is left of center, turn left to center it
            if cx < (width/2 - 25):
                rospy.loginfo("left turn" + str(cx))
                self.landmarkDrive = False
                self.landmarkLeft = True
            #if centroid is right of center, turn right to center it
            elif cx > (width/2 + 25):
                rospy.loginfo("right turn" + str(cx))
                self.landmarkDrive = False
                self.landmarkRight = True
            #if centroid is in center of image, drive forwards
            else:
                rospy.loginfo("drive" + str(cx))
                self.landmarkLeft = False
                self.landmarkRight = False
                self.landmarkDrive = True
        
        #display original, hsv view and mask
        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        #wait for new input
        cv2.waitKey(3)

    #defines what to do when arrow keys are used for teleoperation
    def on_press(self, key):
        #if a key is pressed, set the keyPressed flag to True (used in our init for precedence) and publish the appropriate movement command
        if(key == Key.up):
            self.keyPressed = True
            self.move_cmd.linear.x = 0.5 #move forwards
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)
        if(key == Key.down):
            self.keyPressed = True
            self.move_cmd.linear.x = -0.5 #move backwards
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)
        if(key == Key.left):
            self.keyPressed = True
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 1 #turn left
            self.cmd_vel.publish(self.move_cmd)
        if(key == Key.right):
            self.keyPressed = True
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = -1 #turn right
            self.cmd_vel.publish(self.move_cmd)

    #when a key is relesed, set the keyPressed flag to false, effectively exiting teleoperation
    def on_release(self, key):
        self.keyPressed = False

#initializes project and catches any errors
if __name__ == '__main__':
    try:
        project1()
        rospy.loginfo("project1")
    except:
        rospy.loginfo("project1 node terminated.")
