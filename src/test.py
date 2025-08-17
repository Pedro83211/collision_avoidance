from dis import dis
import rospy
import roslib            
import actionlib
from cola2.utils.ned import NED
import matplotlib.pyplot as plt
import math
from math import *
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32
from cola2_msgs.msg import GoalDescriptor, CaptainStatus, CaptainStateFeedback
from cola2_msgs.msg import PilotActionResult, PilotAction, PilotGoal
from cola2_msgs.msg import NavSts
from cola2_msgs.msg import BodyVelocityReq
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import split
import numpy as np
import copy
from geometry_msgs.msg import PolygonStamped, Point32

side = math.sqrt(400)/2
danger_zone_coords = [[-side/3,-side], [-side/3,side], [side/3,side], [side/3,-side], [-side/3,-side]]
danger_polygon = Polygon(danger_zone_coords)

danger_zone_array = []

for i in range(math.ceil(6/2 - 1)):
    split_line = LineString([(-side, side - side*(i + 1)/2), (side, side - side*(i + 1)/2)])
    danger_zone_array.append(split(danger_polygon, split_line).geoms[1])
    danger_polygon = split(danger_polygon, split_line).geoms[0]

print(danger_zone_array)
danger_zone = Polygon(danger_zone_array[math.floor((5) / 2 - 1)])
danger_zone = danger_zone.buffer(1.0, join_style=2)