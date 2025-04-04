#!/usr/bin/env python
import rospy
from cola2_msgs.msg import  NavSts

#import classes
from robot import Robot

class CollisionAvoidance:

    def __init__(self, name):
        self.name = name
        self.robot_position_north=0
        self.robot_position_east=0
        # self.gps_status = 0
         # Get config parameters from the parameter server  
        self.robot_handler = Robot("sparus_1")

        rospy.Subscriber(
            '/sparus_1/navigator/navigation',
            NavSts,
            self.update_robot_position) 

        # rospy.Subscriber(
        #     '/sparus_1/navigator/gps',
        #     NavSts,
        #     self.update_robot_position) 
        
        rospy.sleep(10)

        self.robot_handler.send_section_strategy((self.robot_position_north, self.robot_position_east), (10, 0), 1)
    
    def update_robot_position(self, msg):
        self.robot_position_north = msg.position.north
        self.robot_position_east = msg.position.east

    # def update_gps_status(self, msg):
    #     self.gps_status = msg.service

if __name__ == '__main__':
    try:
        rospy.init_node('collision_avoidance')
        AUVs = CollisionAvoidance(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass