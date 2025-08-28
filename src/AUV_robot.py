#!/usr/bin/env python
import rospy
from cola2_msgs.msg import  NavSts
import xml.etree.ElementTree as ET

#import classes
from robot import Robot
from mission_generator import GenerateMission

class CollisionAvoidance:

    def __init__(self, name):
        self.name = name
        self.robot_ID = self.get_param('~robot_ID')
        self.mission_path = self.get_param('mission_path',"/home/pedro/catkin_ws/src/collision_avoidance/missions/mission_" + str(self.robot_ID) +".xml")
        self.robot_position = 0
        self.robot_handler = Robot("robot")
        self.mission_handler = GenerateMission("mission")
        self.last_section = False

        rospy.Subscriber(
            '/sparus_' + str(self.robot_ID) + '/navigator/navigation',
            NavSts,
            self.update_robot_position) 
        
        rospy.sleep(10)

        self.mission_steps()

    def mission_steps(self):
        try:
            # Parse the XML file
            tree = ET.parse(self.mission_path)
            root = tree.getroot()

            # Recorrer cada "mission_step" en el XML
            for mission_step in root.findall("mission_step"):
                maneuver = mission_step.find("maneuver")

                last_str = maneuver.find("last_section")
                last = bool(int(last_str.text))

                # Extraer final_latitude y final_longitude si están presentes
                initial_point = self.robot_handler.ned2geodetic(self.robot_position)

                final_lat = maneuver.find("final_latitude")
                final_lon = maneuver.find("final_longitude")
                final_point = (float(final_lat.text), float(final_lon.text))

                if final_lat is not None and final_lon is not None:
                    self.robot_handler.send_section_strategy(initial_point, final_point, last)

        except ET.ParseError as e:
            print(f"Error al analizar el archivo XML: {e}")
        except Exception as e:
            print(f"Error inesperado: {e}")
        
    def update_robot_position(self, msg):
        self.robot_position = (msg.position.north, msg.position.east) 

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
        init_time = rospy.get_time()
        AUVs = CollisionAvoidance(rospy.get_name())
        print("Mission completed in: " + str(rospy.get_time() - init_time) + " seconds")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass