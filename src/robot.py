#!/usr/bin/env python
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
from shapely.geometry import Polygon, Point
import numpy as np

class Robot:

    def __init__(self, name):
        self.name = name
        self.tolerance = self.get_param('tolerance',1)
        self.surge_velocity = self.get_param('surge_velocity',0.5)
        self.battery_topic = self.get_param('~battery_topic','/sparus_1/batteries/status')
        self.section_action = self.get_param('~section_action','/sparus_1/pilot/world_section_req') 
        self.section_result = self.get_param('~section_result','/sparus_1/pilot/world_section_req/result') 
        self.number_of_robots = self.get_param('number_of_robots')
        self.navigation_depth = self.get_param('~navigation_depth',0)
        self.ned_origin_lat = self.get_param('ned_origin_lat',39.543330)
        self.ned_origin_lon = self.get_param('ned_origin_lon',2.377940)
        self.robot_ID = self.get_param('~robot_ID',0)
        self.robot_name = self.get_param('~robot_name','sparus')
        self.collision_algorithm = self.get_param('collision_algorithm', 'stop&wait')
        self.distance = []
        self.travelled_distance = []
        self.robots_travelled_distances = [0,0,0,0,0,0]
        self.robot_alive = False
        self.is_section_actionlib_running = False
        self.battery_status = [0,0,0]
        self.first_time = True
        self.ns = rospy.get_namespace()
        self.ned = NED(self.ned_origin_lat, self.ned_origin_lon, 0.0)  #NED frame
        self.section_req = PilotGoal()
        self.collision = False
        robot_data = [0,0]
        self.robots_position = []
        self.robots_orientation = []
        self.collision_check = []
        self.critical_dist = 10.0
        self.arrived = True
        self.last = False
        # Define the main polygon object
        self.danger_zone_coords = ((10,20),(-10,20),(10,-20),(10,-20))
        for robot in range(self.number_of_robots):
            self.robots_position.append(robot_data) #set the self.robots_position initialized to 0
            self.robots_orientation.append([0]) #set the self.robots_orientation initialized to 0
            self.collision_check.append(False)

        # Publishers
        self.body_velocity_req_pub = rospy.Publisher('/sparus_' + str(self.robot_ID) + '/controller/body_velocity_req',
                                         BodyVelocityReq,
                                         queue_size=1)     #'/robot'+str(self.robot_ID)+'/travelled_distance' ,
        
        # Subscribers
        for robot in range(self.number_of_robots):
            rospy.Subscriber(
                '/sparus_' + str(robot + 1) + '/navigator/navigation',
                NavSts,
                self.update_robot_position)

        #Actionlib section client
        self.section_strategy = actionlib.SimpleActionClient('/sparus_' + str(self.robot_ID) + '/pilot/actionlib',PilotAction) #'/sparus_'+str(self.robot_ID)+'/pilot/actionlib',PilotAction
        self.section_strategy.wait_for_server()
        
        # Init periodic timers
        rospy.Timer(rospy.Duration(0.03), self.check_collision)

    def send_section_strategy(self,initial_point,final_point,robot_id,last):

        init_lat, init_lon = initial_point
        final_lat, final_lon = final_point

        self.section_req = PilotGoal()
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
        self.success_result = False
        self.is_section_actionlib_running = True
        self.section_strategy.send_goal(self.section_req)
        
        #  Wait for result or cancel if timed out
        self.section_strategy.wait_for_result()

        # In case of collision, waits for the AUV to arrive to goal
        self.wait_for_arrival()

        self.first_time = False
        
    def update_robot_position(self, msg):
        try:
            frame_id = int(msg.header.frame_id[7])
        except:
            pass
        else:
            self.robots_position[frame_id - 1] = (msg.position.north, msg.position.east)
            self.robots_orientation[frame_id - 1] = msg.orientation.yaw

    def check_collision(self, event):
        # Sets danger zone based on the algorithm used
        if self.collision_algorithm == 'stop&wait':
            self.danger_zone = Polygon(self.danger_zone_coords)
        elif self.collision_algorithm == 'PF':
            self.danger_zone = Point(self.robots_position[self.robot_ID - 1]).buffer(self.critical_dist)

        for robot in range(self.number_of_robots):
            if (robot == self.robot_ID - 1): continue
            if (self.danger_zone.contains(Point(self.robots_position[robot])) and not self.first_time):
                self.collision_check[robot] = True
            else: self.collision_check[robot] = False

        #print(self.collision_check)

        if any(self.collision_check) or not self.arrived:
            self.collision = True
            self.arrived = False
            self.section_strategy.cancel_all_goals()
            self.avoid_collision()
        elif (self.collision and self.collision_algorithm == 'stop&wait'):
                self.collision = False
                self.section_strategy.send_goal(self.section_req)
                self.section_strategy.wait_for_result()

    def avoid_collision(self):

        if(self.collision_algorithm == 'PF'):

            w1 = 1.0
            w2 = 2.0

            goal_pos_x, goal_pos_y, _ = self.ned.geodetic2ned([self.section_req.final_latitude, self.section_req.final_longitude, 0])
            goal_pos = np.array([goal_pos_x, goal_pos_y])
            init_pos = np.array(self.robots_position[self.robot_ID - 1])

            #Checks if AUV is at goal
            goal_zone = Point(goal_pos).buffer(2)
            if(goal_zone.contains(Point(init_pos)) and not self.last):
                self.arrived = True
            else: self.arrived = False

            goal_vector = self.unit_vector(init_pos, goal_pos)

            obs_vector = self.obstacle_vector(init_pos)

            final_vector = w1 * goal_vector + w2 * obs_vector

            angle = math.atan2(final_vector[1], final_vector[0])

            ang_err = self.angle_correction(angle)

            Wz = ang_err * 0.7

            if (abs(ang_err) > pi/4): Vx = 0
            else: Vx = self.surge_velocity

            # publish the data
            self.send_body_velocity_req(Vx, Wz)

        else: 
            print("Within critical distance")

    def unit_vector(self, init_pos, final_pos):
        vector = final_pos - init_pos
        magnitude = np.linalg.norm(vector)
        return vector/magnitude

    def obstacle_vector(self, init_pos):

        obs_vector = np.array([0.0, 0.0])

        for robot in range(self.number_of_robots):
            if (robot == self.robot_ID - 1): continue
            if self.collision_check[robot]:
                obs_pos = np.array(self.robots_position[robot])
                obs_dist = np.linalg.norm(init_pos - obs_pos)

                K = (self.critical_dist - obs_dist) / self.critical_dist
                print("Within critical distance")

                obs_vector += K * self.unit_vector(obs_pos, init_pos)

        return obs_vector

    def angle_correction(self, angle):

        if(abs(angle) > pi/2 and abs(self.robots_orientation[self.robot_ID - 1]) > pi/2):
            if(self.robots_orientation[self.robot_ID - 1] < 0 and angle > 0):
                return angle - (self.robots_orientation[self.robot_ID - 1] + 2 * pi)
            elif (self.robots_orientation[self.robot_ID - 1] > 0 and angle < 0):
                return angle - (self.robots_orientation[self.robot_ID - 1] - 2 * pi)
            
        return angle - self.robots_orientation[self.robot_ID - 1]


    def wait_for_arrival(self):
        while not rospy.is_shutdown():
            if self.arrived:
                break

    #Args are: [header.seq header.stamp header.frame_id goal.requester goal.priority 
    #           twist.linear.x twist.linear.y twist.linear.z twist.angular.x twist.angular.y twist.angular.z 
    #           disable_axis.x  .y disable_axis.z disable_axis.roll disable_axis.pitch disable_axis.yaw]
    def send_body_velocity_req(self, Vx, Wz):
        msg = BodyVelocityReq()
        msg.header.stamp = rospy.Time.now()
        msg.goal.requester = rospy.get_name()
        msg.goal.priority = GoalDescriptor.PRIORITY_SAFETY_HIGH
        msg.twist.linear.x = Vx
        msg.twist.angular.z = Wz
        self.body_velocity_req_pub.publish(msg)

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