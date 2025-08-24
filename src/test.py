from dis import dis
import rospy
import roslib            
import actionlib
from cola2.utils.ned import NED
import matplotlib.pyplot as plt
import math
from math import *
from cola2_msgs.msg import GoalDescriptor
from cola2_msgs.msg import PilotAction, PilotGoal
from cola2_msgs.msg import NavSts
from cola2_msgs.msg import BodyVelocityReq
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import split
import numpy as np
import copy
from geometry_msgs.msg import PolygonStamped, Point32


        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # ++++++++++++++++++++++++++++SYSTEM CONSTANTS++++++++++++++++++++++++++++
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tolerance = 1
surge_velocity = 0.5
number_of_robots = 5
navigation_depth = 0
ned_origin_lat = 39.543330
ned_origin_lon = 2.377940
robot_ID = 5
robot_name = 'sparus_3'
collision_algorithm = 'stop&wait'
area_of_exploration = 400
coef = 3
critical_dist = 7.0
w1 = 1.0
w2 = 2.0
k_th = 0.7

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++SYSTEM VARIABLES++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ns = rospy.get_namespace()
ned = NED(ned_origin_lat, ned_origin_lon, 0.0)  #NED frame
section_req = PilotGoal()
robot_data = [0,0]
robots_position = []
robots_orientation = []
collision_check = []
collision_pos = []
arrived = True
first_section = True
first_collision = True
once = [True, True]
last = False

# Initialize arrays
for robot in range(number_of_robots):
    robots_position.append(robot_data) #set the robots_position initialized to 0
    robots_orientation.append([0]) #set the robots_orientation initialized to 0
    collision_check.append(False)
          
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++DANGER ZONE INITIALIZATION+++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
if collision_algorithm == 'stop&wait':
    half_side = math.sqrt(area_of_exploration)/2
    danger_zone_coords = [[-half_side/coef,-half_side], [-half_side/coef,half_side], [half_side/coef,half_side], 
                               [half_side/coef,-half_side], [-half_side/coef,-half_side]]
    danger_polygon = Polygon(danger_zone_coords)
    danger_zone = danger_polygon

# Split danger zone depending on the number of robots             
if number_of_robots > 3:
    zones_num = math.ceil(number_of_robots/2)
    if(number_of_robots % 2 != 0):
        if(robot_ID % 2 != 0):
            zones_num -= 1
        danger_zone_array = []
        for i in range(number_of_robots):
            split_line = LineString([(-half_side, half_side - half_side*2*(math.floor(i/2) + 1)/zones_num), 
                                     (half_side, half_side - half_side*2*(math.floor(i/2) + 1)/zones_num)])
            
            try:
                danger_zone_array.append(split(danger_polygon, split_line).geoms[1])
                danger_polygon = split(danger_polygon, split_line).geoms[0]
            except:
                danger_zone_array.append(split(danger_polygon, split_line).geoms[0])
            if(robot_ID - 1 == number_of_robots):
                danger_zone = Polygon(danger_zone_array[-1]).buffer(tolerance, join_style=2)
                break
        print(danger_zone)