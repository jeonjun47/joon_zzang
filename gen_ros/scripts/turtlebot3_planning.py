#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys,os
import rospy
import tf
import threading
from geometry_msgs.msg import Twist
from math import *
from move_base_msgs.msg import MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from numpy import asarray
from matplotlib import pyplot as plt

class turtlebot3_planning():
    def __init__(self):
        rospy.init_node('path_planner', anonymous=False)
        
        # self.odom = Odometry()
        # self.position = self.odom.pose.pose.position
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.pose_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        
        self.rate = rospy.Rate(10)
        ## 경로점
        self.waypoint_x = [5, 5, 0, 0]
        self.waypoint_y = [0, 5, 5, 0]
        self.point = 0
        
        self.current_x = 0
        self.current_y = 0
        
        self.yaw = 0
        
        self.logPosX = []
        self.logPosY = []
        
        self.pid_controller()
        
        
        print('next point : {}, {}'.format(self.waypoint_x[0], self.waypoint_y[0]))
        
        self.move2goal()
        
        ## 현재 위치 값
    def pose_callback(self, data):
        self.current_position = data.pose.pose.position
        self.current_angle = data.pose.pose.orientation
        self.transform = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])

        self.yaw = self.transform[2]
        
        self.current_x = round(self.current_position.x,4)
        self.current_y = round(self.current_position.y,4)

    def desire_yaw(self, waypoint_x, waypoint_y):
        return atan2(waypoint_y - self.current_y, waypoint_x - self.current_x)    
    

    def error(self,waypoint_x,waypoint_y):
        if((self.desire_yaw(waypoint_x,waypoint_y) - self.yaw) > 3.14):
            err = self.desire_yaw(waypoint_x,waypoint_y) - self.yaw - 6.28
        elif((self.desire_yaw(waypoint_x,waypoint_y) - self.yaw) < - 3.14):
            err = self.desire_yaw(waypoint_x,waypoint_y) - self.yaw + 6.28
        else:
            err = self.desire_yaw(waypoint_x,waypoint_y) - self.yaw

        return err
    
    def distance(self, waypoint_x, waypoint_y):
        return sqrt(pow((waypoint_x - self.current_x),2) + pow((waypoint_y - self.current_y),2))    
    
    def pid_controller(self):
        
        self.k_p = 1.5      #1.0 ~ 2.0
        self.k_i = 0.01     #0.01 ~ 0.1
        self.k_d = 0.00005   #0.00001 ~ 0.0001
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.0333
        self.output = 0
        
        # self.p_control = self.k_p * self.error(waypoint_x[self.point], waypoint_y[self.point])
        # self.p_control = self.k_p * self.error(waypoint_x[self.point], waypoint_y[self.point])
        # self.i_control += self.k_i * self.error * self.controlTime
        # self.d_control = self.k_d * (self.error - self.prev_error)/self.controlTime
        
        # self.output = self.p_control + self.i_control + self.d_control
        
        # self.prev_error = self.error     
    
    def move2goal(self):
        self.cmd_vel = Twist()
        _tempCount = 0
        i = 0

        while self.distance(self.waypoint_x[self.point],self.waypoint_y[self.point]) >= 0.5:
            self.rate.sleep()
            
            if (_tempCount % 5) == 0:
                self.logPosX.append(self.current_x)
                self.logPosY.append(self.current_y)
            
            _tempCount += 1
            
            self.cmd_vel.linear.x = 0.5
            self.cmd_vel.linear.y = 0
            self.cmd_vel.linear.z = 0
            
            self.error_func = self.error(asarray(self.waypoint_x[self.point]), asarray(self.waypoint_y[self.point]))
            # self.error_func2 = self.error(self.waypoint_x[self.point], self.waypoint_y[self.point])
            self.p_control = self.k_p * self.error_func
            self.i_control += self.k_i * self.error_func * self.controlTime
            self.d_control = self.k_d * -(self.error_func - self.prev_error)/self.controlTime
            
            self.pi_control = self.p_control + self.i_control
            
            self.output = self.p_control + self.i_control + self.d_control
            
            self.prev_error = self.pi_control
            
            
            self.cmd_vel.angular.x = 0
            self.cmd_vel.angular.y = 0
            # self.cmd_vel.angular.z = self.output
            self.cmd_vel.angular.z = self.output
            
                        
            # # 예외 처리
            # if self.cmd_vel.angular.z >= 30:
            #     self.cmd_vel.angular.z = 30
                
            # elif self.cmd_vel.angular.z <= -30:
            #     self.cmd_vel.angular.z = -30
             
            # else:
            #     self.cmd_vel.angular.z = self.cmd_vel.angular.z
                
            self.cmd_pub.publish(self.cmd_vel)
            
            # print('desire yaw : {}, current yaw : {}'.format(self.desire_yaw(self.waypoint_x[self.point], self.waypoint_y[self.point]), self.yaw))
            
            if self.distance(self.waypoint_x[self.point], self.waypoint_y[self.point]) < 0.5 and self.point < len(self.waypoint_x) - 1:
                
                self.point += 1
                print('count : {}'.format(self.point))
                print('next point : {}, {}'.format(self.waypoint_x[self.point], self.waypoint_y[self.point]))
                
                
                
            elif(self.point == len(self.waypoint_x)):
                self.point = 0   
                    
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.cmd_pub.publish(self.cmd_vel)
        _n = 0
        _point = 0
        for _x in self.waypoint_x:
            _y = self.waypoint_y[_point]
            plt.scatter(_x,_y, s=10, c='blue')
            _point += 1 
        for x in self.logPosX:
            y = self.logPosY[_n]
            plt.scatter(x,y, s=2, c='red')
            plt.pause(0.01)
            _n += 1
            
        print('---Finish---')
        plt.show()    
            # if (self.point == len(self.waypoint_x)-1):
            #     self.point = 0

if __name__ == '__main__':
    try:
        go_to = turtlebot3_planning()
    except rospy.ROSInterruptException:
        pass
    