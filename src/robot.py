#!/usr/bin/env python
from dis import dis
import rospy
import roslib            
import actionlib
from cola2.utils.ned import NED
import matplotlib.pyplot as plt
from math import *
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32
from cola2_msgs.msg import GoalDescriptor, CaptainStatus, CaptainStateFeedback
from cola2_msgs.msg import PilotActionResult, PilotAction, PilotGoal
from cola2_msgs.msg import NavSts
from cola2_msgs.msg import BodyVelocityReq
from cola2_msgs.srv import Goto, GotoRequest, Section, SectionRequest
from shapely.geometry import Polygon
from sensor_msgs.msg import BatteryState
import numpy as np
from std_srvs.srv import Empty, EmptyResponse
from multi_robot_system.msg import TravelledDistance, CoverageStartTime

class Robot:

    def __init__(self, name):
        self.name = name
        self.tolerance = self.get_param('tolerance',2)
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
        self.robots_information = []
        for robot in range(self.number_of_robots):
            self.robots_information.append(robot_data) #set the self.robots_information initialized to 0

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
        rospy.Timer(rospy.Duration(0.05), self.check_collision)

    def send_section_strategy(self,initial_point,final_point,robot_id,last):
        initial_position_x = initial_point[0]
        final_position_x = final_point[0]
        initial_position_y = initial_point[1]
        final_position_y = final_point[1]

        init_lat, init_lon, _ = self.ned.ned2geodetic([initial_position_x, initial_position_y, 0.0])
        final_lat, final_lon, _ = self.ned.ned2geodetic([final_position_x, final_position_y, 0.0])

        self.section_req = PilotGoal()
        self.section_req.initial_latitude = init_lat
        self.section_req.initial_longitude = init_lon
        self.section_req.initial_depth = self.navigation_depth
        # section_req.final_yaw = self.robots_information[robot_id][2] #yaw
        self.section_req.final_latitude = final_lat
        self.section_req.final_longitude = final_lon
        self.section_req.final_depth = self.navigation_depth
        self.section_req.final_altitude = self.navigation_depth

        self.section_req.heave_mode = 0
        # uint64 DEPTH=0
        # uint64 ALTITUDE=1
        # uint64 BOTH=2

        # If last section, null tolerance to force maintain position
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

        self.first_time = False

        # section_req = SectionRequest()
        # section_req.initial_x = initial_position_x
        # section_req.initial_y = initial_position_y
        # section_req.initial_depth = self.navigation_depth
        # section_req.final_x = final_position_x
        # section_req.final_y = final_position_y
        # section_req.final_altitude = self.navigation_depth
        # section_req.final_depth = self.navigation_depth
        # section_req.reference = 0
        # section_req.heave_mode = 0
        # section_req.surge_velocity = self.surge_velocity
        # section_req.tolerance_xy = 1
        # section_req.timeout = 6000
        # section_req.no_altitude_goes_up = 0
        # self.section_srv(section_req)
        
    def update_robot_position(self, msg):
        frame_id = msg.header.frame_id
        if frame_id == "sparus_1/base_link":
            self.robots_information[0] = (msg.position.north, msg.position.east)
        elif frame_id == "sparus_2/base_link":
            self.robots_information[1] = (msg.position.north, msg.position.east)


    def check_collision(self, event):
        if(self.robots_information[0][0] < 10 and
           self.robots_information[0][0] > -10):
            if(self.robot_ID == 2 and not self.first_time):
                self.collision = True
                self.section_strategy.cancel_all_goals()
                self.avoid_collision()
        elif self.collision:
            self.collision = False
            self.section_strategy.send_goal(self.section_req)
            self.section_strategy.wait_for_result()


    #Args are: [header.seq header.stamp header.frame_id goal.requester goal.priority 
    #           twist.linear.x twist.linear.y twist.linear.z twist.angular.x twist.angular.y twist.angular.z 
    #           disable_axis.x  .y disable_axis.z disable_axis.roll disable_axis.pitch disable_axis.yaw]
    def avoid_collision(self):

        # publish the data
        msg = BodyVelocityReq()
        msg.header.stamp = rospy.Time.now()
        msg.goal.requester = rospy.get_name()
        msg.goal.priority = GoalDescriptor.PRIORITY_SAFETY_HIGH
        msg.disable_axis.y = True
        msg.twist.angular.z = 10
        self.body_velocity_req_pub.publish(msg)
  
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