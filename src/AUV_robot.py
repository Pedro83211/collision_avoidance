#!/usr/bin/env python
import rospy
from cola2_msgs.msg import  NavSts

#import classes
from robot import Robot

class CollisionAvoidance:

    def __init__(self, name):
        self.name = name
        self.robot_ID = self.get_param('~robot_ID')
        self.robot_position_north=0
        self.robot_position_east=0
        self.robot_handler = Robot("robot")
        self.last_section = False

        rospy.Subscriber(
            '/sparus_' + str(self.robot_ID) + '/navigator/navigation',
            NavSts,
            self.update_robot_position) 
        
        rospy.sleep(10)

        if self.robot_ID == 1:
            self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (10, 10), self.robot_ID, self.last_section)
            self.last_section = True
            self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (-10, -10), self.robot_ID, self.last_section)
            # self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (10, 0), self.robot_ID, self.last_section)
            # self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (-10, 0), self.robot_ID, self.last_section)
            # self.last_section = True
            # self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (10, 0), self.robot_ID, self.last_section)
        elif self.robot_ID == 2:
            self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (10, -10), self.robot_ID, self.last_section)
            self.last_section = True
            self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (-10, 10), self.robot_ID, self.last_section)
            # self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (-10, 0), self.robot_ID, self.last_section)
            # self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (0, 0), self.robot_ID, self.last_section)
            # self.last_section = True
            # self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (-10, 0), self.robot_ID, self.last_section)
        elif self.robot_ID == 3:
            self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (-10, 10), self.robot_ID, self.last_section)
            self.last_section = True
            self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (10, -10), self.robot_ID, self.last_section)
        elif self.robot_ID == 4:
            self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (-10, -10), self.robot_ID, self.last_section)
            self.last_section = True
            self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (10, 10), self.robot_ID, self.last_section)
        
    def update_robot_position(self, msg):
        self.robot_position_north = msg.position.north
        self.robot_position_east = msg.position.east

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
        AUVs = CollisionAvoidance(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass