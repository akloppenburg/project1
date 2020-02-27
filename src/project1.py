#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan
from pynput import keyboard
from pynput.keyboard import Key, Listener
import random
import math


class project1():

    def __init__(self):

        self.move_cmd = Twist()

        self.isBumped = False
        self.keyPressed = False
        self.isSymmetric = False
        self.isAsymmetricLeft = False
        self.isAsymmetricRight = False

        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        rospy.init_node('project1')

        self.bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.BumperCallback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.LaserCallback)

        rospy.on_shutdown(self.shutdown)

        self.listener = keyboard.Listener(
            on_press = self.on_press,
            on_release = self.on_release)
        self.listener.start()
        
        self.r = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            #if the bumpers are engaged, stop the robot
            if(self.isBumped):
                self.cmd_vel.publish(Twist())
                rospy.loginfo("bumped")
            #if the bumpers are NOT engaged, check for teleoperation
            elif(self.keyPressed):
                continue
                #listener above handles all key input, so this just makes sure that no other Twist commands are sent
            #if bumpers are NOT engaged and no arrow keys are pressed, engage normal movement
            else:
                #if symmetric obstacle is detected, complete full 180 spin and then continue
                if(self.isSymmetric):
                    #turn a full 180, stopping if we bump into anything or we do teleoperation
                    speed = 75
                    angle = 180
                    rospy.loginfo("symmetric detected (0.5m)...halting...")

                    #Converting from angles to radians
                    angular_speed = speed*2*math.pi/360
                    relative_angle = angle*2*math.pi/360

                    #halt the robot
                    self.move_cmd.linear.x=0
                    self.move_cmd.linear.y=0
                    self.move_cmd.linear.z=0
                    self.move_cmd.angular.x=0
                    self.move_cmd.angular.y=0

                    #begin rotation
                    rospy.loginfo("rotating...")
                    self.move_cmd.angular.z=angular_speed

                    #setting the current time for distance calculus
                    t0 = rospy.Time.now().to_sec()
                    current_angle = 0
                    while (current_angle < relative_angle):
                        rospy.loginfo("looping")
                        self.cmd_vel.publish(self.move_cmd)
                        t1 = rospy.Time.now().to_sec()
                        current_angle = angular_speed*(t1-t0)

                    #force our robot to stop
                    rospy.loginfo("rotation complete.")   
                    self.move_cmd.angular.z = 0
                    self.cmd_vel.publish(Twist())
                                    
                            
                #if asymmetric obstacle is detected on the left, veer right
                elif(self.isAsymmetricLeft):
                    rospy.loginfo("avoid left")
                    self.move_cmd.linear.x = 0.5
                    self.move_cmd.angular.z = -1
                    self.cmd_vel.publish(self.move_cmd)
                #if asymmetric obstacle is detected on the right, veer left
                elif(self.isAsymmetricRight):
                    rospy.loginfo("avoid right")
                    self.move_cmd.linear.x = 0.5
                    self.move_cmd.angular.z = 1
                    self.cmd_vel.publish(self.move_cmd)
                #otherwise, drive forwards and turn every meter
                else:
                    rospy.loginfo("moving forward (normal)...")
                    #drive one meter, stopping if any other events occur
                    self.drive(0.3, 0.3)
                    rospy.loginfo("drove")
                    #turn randomly, stopping if any bump or key press events occur
                    random_turn = random.uniform(-15, 15) #in degrees
                    self.turn(random_turn, 5)
                    rospy.loginfo("turned " + str(random_turn) + " degrees")
                    
                
        self.r.sleep()
                        
        
    def shutdown(self):
        rospy.loginfo("Stop TurtleBot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def turn(self, angle, speed):

        if(self.isBumped or self.keyPressed):
            rospy.loginfo("turning machine broke")
            return

        velocity_msg = Twist()

        #Converting from angles to radians
        angular_speed = speed*2*math.pi/360
        relative_angle = angle*2*math.pi/360

        velocity_msg.linear.x = 0
        velocity_msg.angular.z = angular_speed
        
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        
        while(current_angle < relative_angle):
            if(self.isBumped or self.keyPressed):
                rospy.loginfo("turning machine broke")
                return
            self.cmd_vel.publish(velocity_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)
            
        #Forcing our robot to stop
        self.cmd_vel.publish(Twist())


    def drive(self, distance, speed):

        if(self.isBumped or self.keyPressed or self.isSymmetric or self.isAsymmetricLeft or self.isAsymmetricRight):
            rospy.loginfo("driving machine broke")
            return

        velocity_msg = Twist()
        velocity_msg.angular.z = 0
        velocity_msg.linear.x = speed
        
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        
        while(current_distance < distance):
            if(self.isBumped or self.keyPressed or self.isSymmetric or self.isAsymmetricLeft or self.isAsymmetricRight):
                rospy.loginfo("driving machine broke")
                return
            self.cmd_vel.publish(velocity_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = speed*(t1-t0)
            
        #Forcing our robot to stop
        self.cmd_vel.publish(Twist())


    def BumperCallback(self, data):
        if (data.state == 1):
            self.isBumped = True
            rospy.loginfo("bumped")
        elif (data.state == 0):
            self.isBumped = False
            rospy.loginfo("un-bumped")
        #else:
        #    self.isBumped = False
    
    #640 values from our laser
    #middle 108(straight ahead and +- about 15 degrees) are used for symmetric detection
    #rest are used for automatic avoidance and escaping
    def LaserCallback(self, data):
        #set values as false, only changing them if there is a symmetric or asymmetric obstacle within 1 foot
        self.isSymmetric = False
        self.isAsymmetricLeft = False
        self.isAsymmetricRight = False
        #symmetric detection
        for i in range(266, 374):
            print "distance is:", data.ranges[320]
            if(data.ranges[i] <= 0.5):
                self.isSymmetric = True
                break
            else:
                self.isSymmetric = False
        #asymmetric detection
        for i in range(0, 265):
            if(data.ranges[i] <= 0.3):
                self.isAsymmetricLeft = True
        for i in range(375, 639):
            if(data.ranges[i] <= 0.3):
                self.isAsymmetricRight = True

    #defines what to do when arrow keys are used for teleoperation
    def on_press(self, key):
        if(key == Key.up):
            self.keyPressed = True
            self.move_cmd.linear.x = 0.5
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)
        if(key == Key.down):
            self.keyPressed = True
            self.move_cmd.linear.x = -0.5
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)
        if(key == Key.left):
            self.keyPressed = True
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 1
            self.cmd_vel.publish(self.move_cmd)
        if(key == Key.right):
            self.keyPressed = True
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = -1
            self.cmd_vel.publish(self.move_cmd)

    def on_release(self, key):
        self.keyPressed = False

 
if __name__ == '__main__':
    #try:
        project1()
        rospy.loginfo("project1")
    #except:
        rospy.loginfo("project1 node terminated.")
