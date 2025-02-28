#!/usr/bin/env python
import rospy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import datetime
from numpy import *
import numpy as np
import os
import subprocess
import pickle
from shapely.geometry import Polygon,Point
from geometry_msgs.msg  import PointStamped
from std_srvs.srv import Empty
from geometry_msgs.msg import  PolygonStamped, Point32, Polygon
from cola2_msgs.msg import  NavSts, CaptainStateFeedback, PilotActionResult
from collision_avoidance.msg import CoverageStartTime, ExplorationUpdate, RegularObjectInformation,PriorityObjectInformation 
from std_msgs.msg import Int16, Bool

#import classes
from robot import Robot

class CollisionAvoidance:

    def __init__(self, name):
        self.name = name
         # Get config parameters from the parameter server
        self.robot_ID = self.get_param('~robot_ID')   
        self.number_of_robots = self.get_param('number_of_robots')
        self.number_of_auvs = self.get_param('number_of_auvs')
        self.robot_handler = Robot("robot")
        self.robot_initialization = np.array([])
        self.empty_array=np.array([])
        self.robots_information =[]
        self.robot_data = [0,0]

                # initialize the robots variables
        for i in range(self.number_of_auvs):
            self.robot_initialization = np.append(self.robot_initialization,False) 
            self.actual_sections.append([i,0])
            self.robots_information.append (self.robot_data)
        
        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

        #Subscribers       
        rospy.Subscriber(
            '/robot'+str(self.robot_ID)+'/navigator/navigation',
            NavSts,
            self.update_robot_position) 
        
        rospy.Subscriber(
            '/mrs/coverage_init',
            Bool,
            self.object_exploration_flag)

        self.initialization() 

    def object_exploration_flag(self,msg):
        self.object_exploration = msg.data

    def update_robot_position(self, msg):
        self.robot_position_north = msg.position.north
        self.robot_position_east = msg.position.east   

    def initialization(self): 
        rospy.sleep(6)
        if np.all(self.robot_initialization == False):
            for robot in range(self.number_of_auvs):
                self.robot_initialization[robot] = self.robot_handler.is_robot_alive(robot)

        print("             *************************")
        print("                 ROBOT "+str(self.robot_ID)+ " INITIALIZED   ")
        print("             *************************")

        self.coverage()
    
    def coverage(self):
        # get the closest point of the largest polygon side
        final_point = (-20, 0)
        
        self.robot_handler.send_section_strategy((self.robot_position_north,self.robot_position_east),final_point,self.robot_ID)

    def get_param(self, param_name, default = None):
        if rospy.has_param(param_name):
            param_value = rospy.get_param(param_name)
            return param_value
        elif default is not None:
            return default
        else:
            rospy.logfatal('[%s]: invalid parameters for %s in param server!', self.name, param_name)
            rospy.logfatal('[%s]: shutdown due to invalid config parameters!', self.name)
            exit(0)

if __name__ == '__main__':
    try:
        rospy.init_node('collision_avoidance')
        return_to_NED = CollisionAvoidance(rospy.get_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass