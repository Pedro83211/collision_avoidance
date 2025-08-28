#!/usr/bin/env python
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

class Robot:

    def __init__(self, name):

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # ++++++++++++++++++++++++++++SYSTEM CONSTANTS++++++++++++++++++++++++++++
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        self.name = name
        self.tolerance = self.get_param('tolerance',1)
        self.surge_velocity = self.get_param('surge_velocity',0.5) 
        self.number_of_robots = self.get_param('number_of_robots')
        self.navigation_depth = self.get_param('~navigation_depth',0)
        self.ned_origin_lat = self.get_param('ned_origin_lat',39.543330)
        self.ned_origin_lon = self.get_param('ned_origin_lon',2.377940)
        self.robot_ID = self.get_param('~robot_ID',0)
        self.robot_name = self.get_param('~robot_name','sparus')
        self.collision_algorithm = self.get_param('collision_algorithm', 'stop&wait')
        self.area_of_exploration = self.get_param('area_of_exploration', 400)
        self.coef = self.get_param('reduction_coefficient', 3)
        self.critical_dist = self.get_param('critical_distance', 7.0)
        self.w1 = self.get_param('goal_weight', 1.0)
        self.w2 = self.get_param('obstacle_weight', 2.0)
        self.k_th = self.get_param('k_th', 0.7)

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # ++++++++++++++++++++++++++++SYSTEM VARIABLES++++++++++++++++++++++++++++
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        self.ns = rospy.get_namespace()
        self.ned = NED(self.ned_origin_lat, self.ned_origin_lon, 0.0)  #NED frame
        self.section_req = PilotGoal()
        robot_data = [0,0]
        self.robots_position = []
        self.robots_orientation = []
        self.obs_detect = []
        self.arrived = True
        self.first_section = True
        self.first_collision = True
        self.once = [True, True]
        self.last = False

        # Initialize arrays
        for robot in range(self.number_of_robots):
            self.robots_position.append(robot_data) #set the self.robots_position initialized to 0
            self.robots_orientation.append([0]) #set the self.robots_orientation initialized to 0
            self.obs_detect.append(False)
          
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # +++++++++++++++++++++++DANGER ZONE INITIALIZATION+++++++++++++++++++++++
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        if self.collision_algorithm == 'stop&wait':
            half_side = math.sqrt(self.area_of_exploration)/2
            self.danger_zone_coords = [[-half_side/self.coef,-half_side], [-half_side/self.coef,half_side], [half_side/self.coef,half_side], 
                                       [half_side/self.coef,-half_side], [-half_side/self.coef,-half_side]]
            danger_polygon = Polygon(self.danger_zone_coords)

            # Split danger zone depending on the number of robots             
            if self.number_of_robots > 2:

                danger_zone_array = []
                zones_num = math.ceil(self.number_of_robots/2)
                
                if (self.number_of_robots % 2 == 0):
                    index = math.floor((self.robot_ID) / 2)
                else:
                    if (self.robot_ID % 2 != 0):
                        zones_num -= 1
                        index = (self.robot_ID - 1) / 2
                    else:
                        index = self.robot_ID / 2

                for i in range(zones_num - 1):
                    split_line = LineString([(-half_side, half_side - half_side*2*(i + 1)/zones_num), (half_side, half_side - half_side*2*(i + 1)/zones_num)])
                    danger_zone_array.append(split(danger_polygon, split_line).geoms[1])
                    danger_polygon = split(danger_polygon, split_line).geoms[0]
                
                danger_zone_array.append(danger_polygon)

                # Each row of AUV's has one danger zone assigned to it

                self.danger_zone = Polygon(danger_zone_array[int(index)]).buffer(self.tolerance, join_style=2)

            else:
                self.danger_zone = danger_polygon.buffer(self.tolerance, join_style=2)


        elif self.collision_algorithm == 'PF':
            self.danger_zone = Polygon()

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # ++++++++++++++++++++++++ROS COMPONENTS DEFINITION+++++++++++++++++++++++
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # Publishers
        self.body_velocity_req_pub = rospy.Publisher('/sparus_' + str(self.robot_ID) + '/controller/body_velocity_req',
                                         BodyVelocityReq,
                                         queue_size=1)     #'/robot'+str(self.robot_ID)+'/travelled_distance' ,
        
        self.polygon_stamped_pub = rospy.Publisher('/sparus_' + str(self.robot_ID) + '/danger_zone',
                                         PolygonStamped,
                                         queue_size=1)  
        
        # Subscribers
        for robot in range(self.number_of_robots):
            rospy.Subscriber(
                '/sparus_' + str(robot) + '/navigator/navigation',
                NavSts,
                self.update_robot_position)

        #Actionlib section client
        self.section_strategy = actionlib.SimpleActionClient('/sparus_' + str(self.robot_ID) + '/pilot/actionlib',PilotAction) #'/sparus_'+str(self.robot_ID)+'/pilot/actionlib',PilotAction
        self.section_strategy.wait_for_server()
        
        # Init periodic timers
        rospy.Timer(rospy.Duration(0.03), self.check_collision)

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++AUXILIARY FUNCTIONS+++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    def send_section_strategy(self,initial_point,final_point, last):

        init_lat, init_lon = initial_point
        final_lat, final_lon = final_point

        self.section_req.initial_latitude = init_lat
        self.section_req.initial_longitude = init_lon
        self.section_req.initial_depth = self.navigation_depth
        # section_req.final_yaw = self.robots_position[robot_id][2] #yaw
        self.section_req.final_latitude = final_lat
        self.section_req.final_longitude = final_lon
        self.section_req.final_depth = self.navigation_depth
        self.section_req.final_altitude = self.navigation_depth

        self.section_req.heave_mode = 0
        # uint64 DEPTH=0
        # uint64 ALTITUDE=1
        # uint64 BOTH=2

        # If last section, null tolerance to force maintain position
        self.last = last
        if not last: self.section_req.tolerance_xy = self.tolerance
        else: self.section_req.tolerance_xy = 0
        self.section_req.surge_velocity = self.surge_velocity
        self.section_req.controller_type = 0
        # uint64 SECTION=0
        # uint64 ANCHOR=1
        # uint64 HOLONOMIC_KEEP_POSITION=2
        self.section_req.goal.priority = GoalDescriptor.PRIORITY_SAFETY_HIGH
        self.section_req.goal.requester = rospy.get_name()
        self.section_req.timeout = 6000

        # send section goal using actionlib
        self.section_strategy.send_goal(self.section_req)
        
        #  Wait for result or cancel if timed out
        self.section_strategy.wait_for_result()

        # In case of collision, waits for the AUV to arrive to goal
        self.wait_for_arrival()

        self.first_section = False
        
    def update_robot_position(self, msg):
        try:
            frame_id = int(msg.header.frame_id[7])
        except:
            pass
        else:
            self.robots_position[frame_id] = (msg.position.north, msg.position.east)
            self.robots_orientation[frame_id] = msg.orientation.yaw

    def check_collision(self, event):
        # Sets the potential field danger zone
        if self.collision_algorithm == 'PF':
            self.danger_zone = Point(self.robots_position[self.robot_ID]).buffer(self.critical_dist)
        
        # Sends danger zone coordinates to RViz
        self.send_polygon_stamped()

        # Fills a boolean array with collision detection for each robot
        for robot in range(self.number_of_robots):
            if (robot == self.robot_ID): continue
            if (self.danger_zone.contains(Point(self.robots_position[robot])) and not self.first_section):
                self.obs_detect[robot] = True
            else: self.obs_detect[robot] = False

        if any(self.obs_detect) or not self.arrived:
            self.arrived = False
            if(self.first_collision and self.collision_algorithm == 'PF'):
                print("++++++++++++++++++++++++SPARUS " + str(self.robot_ID) + ": " "CANCELLING GOALS++++++++++++++++++++++++")
                self.section_strategy.cancel_goal()
                self.once = [True, True]
                self.first_collision = False
            self.avoid_collision()

    def avoid_collision(self):

        # Saves the positions variables
        goal_pos_x, goal_pos_y, _ = self.ned.geodetic2ned([self.section_req.final_latitude, self.section_req.final_longitude, 0])
        goal_pos = np.array([goal_pos_x, goal_pos_y])
        init_pos = np.array(self.robots_position[self.robot_ID])
        goal_zone = Point(goal_pos).buffer(self.tolerance)

        if (self.collision_algorithm == 'stop&wait'): 

            #Checks if AUV is at goal
            if goal_zone.contains(Point(init_pos)):
                self.arrived = True
            else: 
            
                if (any(self.obs_detect) and not self.danger_zone.contains(Point(self.robots_position[self.robot_ID]))):

                    section_req_aux = copy.copy(self.section_req)

                    section_req_aux.initial_latitude, section_req_aux.initial_longitude = self.ned2geodetic(init_pos)
                    section_req_aux.final_latitude, section_req_aux.final_longitude = self.ned2geodetic(init_pos)
                    section_req_aux.tolerance_xy = 0

                    # Sends maintain position goal once
                    if self.once[0]:
                        print("++++++++++++++++++++++++SPARUS " + str(self.robot_ID) + ": KEEPING POSITION++++++++++++++++++++++++")
                        self.section_strategy.cancel_goal()
                        self.section_strategy.send_goal(section_req_aux)
                        self.once[0] = False
                        self.once[1] = True

                elif not any(self.obs_detect):
                    if self.once[1]:
                        print("++++++++++++++++++++++++SPARUS " + str(self.robot_ID) + ": HEADED TO GOAL++++++++++++++++++++++++") 
                        self.section_strategy.cancel_goal()
                        self.section_req.initial_latitude, self.section_req.initial_longitude = self.ned2geodetic(init_pos)
                        self.section_strategy.send_goal(self.section_req)
                        self.once[1] = False
                        self.once[0] = True

        elif (self.collision_algorithm == 'PF'):

            #Checks if AUV is at goal
            if goal_zone.contains(Point(init_pos)):
                self.arrived = True
            else: 

                goal_vector = self.unit_vector(init_pos, goal_pos)

                obs_vector = self.obstacle_vector(init_pos)

                final_vector = self.w1 * goal_vector + self.w2 * obs_vector

                angle = math.atan2(final_vector[1], final_vector[0])

                ang_err = self.angle_correction(angle)

                Wy = ang_err * self.k_th

                if (abs(ang_err) > pi/4): Vx = 0
                else: Vx = self.surge_velocity - 0.2

                # publish the data
                self.send_body_velocity_req(Vx, Wy)

    def unit_vector(self, init_pos, final_pos):
        vector = final_pos - init_pos
        magnitude = np.linalg.norm(vector)
        return vector/magnitude

    def obstacle_vector(self, init_pos):

        obs_vector = np.array([0.0, 0.0])

        for robot in range(self.number_of_robots):
            if (robot == self.robot_ID): continue
            if self.obs_detect[robot]:
                obs_pos = np.array(self.robots_position[robot])
                obs_dist = np.linalg.norm(init_pos - obs_pos)

                k = (self.critical_dist - obs_dist) / self.critical_dist
                if (k > 0 and self.once[0]):
                    self.once[0] = False
                    print("++++++++++++++++++++++++SPARUS " + str(self.robot_ID) + ": AVOIDING COLLISION++++++++++++++++++++++++")
                elif self.once[1]:
                    self.once[1] = False
                    print("++++++++++++++++++++++++SPARUS " + str(self.robot_ID) + ": HEADED TO GOAL++++++++++++++++++++++++")

                obs_vector += k * self.unit_vector(obs_pos, init_pos)

        return obs_vector

    def angle_correction(self, angle):

        if(abs(angle) > pi/2 and abs(self.robots_orientation[self.robot_ID]) > pi/2):
            if(self.robots_orientation[self.robot_ID] < 0 and angle > 0):
                return angle - (self.robots_orientation[self.robot_ID] + 2 * pi)
            elif (self.robots_orientation[self.robot_ID] > 0 and angle < 0):
                return angle - (self.robots_orientation[self.robot_ID] - 2 * pi)
            
        return angle - self.robots_orientation[self.robot_ID]


    def wait_for_arrival(self):
        while not rospy.is_shutdown():
            if self.arrived:
                break

    #Args are: [header.seq header.stamp header.frame_id goal.requester goal.priority 
    #           twist.linear.x twist.linear.y twist.linear.z twist.angular.x twist.angular.y twist.angular.z 
    #           disable_axis.x  .y disable_axis.z disable_axis.roll disable_axis.pitch disable_axis.yaw]
    def send_body_velocity_req(self, Vx, Wy):
        msg = BodyVelocityReq()
        msg.header.stamp = rospy.Time.now()
        msg.goal.requester = rospy.get_name()
        msg.goal.priority = GoalDescriptor.PRIORITY_SAFETY_HIGH
        msg.disable_axis.y = True
        msg.disable_axis.z = True
        msg.disable_axis.pitch = True
        msg.disable_axis.roll = True
        msg.twist.linear.x = Vx
        msg.twist.angular.z = Wy #ROS roll is system yaw
        self.body_velocity_req_pub.publish(msg)

    def send_polygon_stamped(self):
        msg = PolygonStamped()
        points = []
        for i in range(len(self.danger_zone.exterior.coords.xy[1])): 
            points.append(Point32())
            points[i].x = self.danger_zone.exterior.coords.xy[0][i]
            points[i].y = -self.danger_zone.exterior.coords.xy[1][i]
            points[i].z = self.navigation_depth
        msg.polygon.points = points
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/world"
        self.polygon_stamped_pub.publish(msg)

    def ned2geodetic(self, point):
        lat, lon, _ = self.ned.ned2geodetic([point[0], point[1], 0])
        return (lat,lon)
         

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
        rospy.init_node('robot')
        robot = Robot(rospy.get_name())
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass