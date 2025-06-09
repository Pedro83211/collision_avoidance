import rospy
import math
from cola2.utils.ned import NED
import xml.etree.ElementTree as ET
from lxml import etree

class GenerateMission:
    def __init__(self, name):
        self.name = name
        self.robot_ID = self.get_param('~robot_ID')
        self.number_of_robots = self.get_param('number_of_robots')
        self.ned_origin_lat = self.get_param('ned_origin_lat',39.543330)
        self.ned_origin_lon = self.get_param('ned_origin_lon',2.377940)
        self.ned = NED(self.ned_origin_lat, self.ned_origin_lon, 0.0)  #NED frame
        self.area_of_exploration = self.get_param('area_of_exploration', 400)
        self.mission_path = self.get_param('mission',"/home/pedro/catkin_ws/src/collision_avoidance/missions/mission_" + str(self.robot_ID) + ".xml")
        self.dist_between_paths = 5
        self.cycle_coeficient = 1
        self.east = 0
        self.north_rev = 0
        self.last = 0

        # Set the number of cycles needed to cover the height of the square of exploration knowing that a cycle is
        # formed by 2 movements
        self.number_of_cycles = 2 * math.sqrt(self.area_of_exploration) / self.dist_between_paths

        if self.number_of_robots > 1:
            # Area of exploration gets smaller in proportion to the total number of robots in the system
            if self.number_of_robots % 2 == 0:
                self.cycle_coeficient = self.number_of_robots / 2
            else:
                if self.robot_ID % 2 == 0:
                    self.cycle_coeficient = (self.number_of_robots - 1) / 2
                else:
                    self.cycle_coeficient = (self.number_of_robots + 1) / 2
            
            self.number_of_cycles /= self.cycle_coeficient
        else:
            self.cycle_coeficient = 1
        

        if self.robot_ID % 2 == 0:
            self.east = -math.sqrt(self.area_of_exploration) / 2
            self.north_rev = math.sqrt(self.area_of_exploration) * (self.cycle_coeficient - 2 * (self.robot_ID / 2 - 1)) / (2 * self.cycle_coeficient)              
        else:
            self.east = math.sqrt(self.area_of_exploration) / 2
            self.north_rev = math.sqrt(self.area_of_exploration) * (self.cycle_coeficient - 2 * ((self.robot_ID + 1) / 2 - 1)) / (2 * self.cycle_coeficient)

        missionElem = etree.Element("mission")

        doc = etree.ElementTree(missionElem)
        
        versionElem = etree.SubElement(missionElem, "version")
        versionElem.text = "2.0"

        for j in range(int(self.number_of_cycles) + 1):
            rem = j % 4 # There are 4 types of movements: to the right, down from the right, to the left, down to the left
            if j != 0:
                if rem == 1:
                    if self.robot_ID % 2 == 0:
                        self.east += 5 * math.sqrt(self.area_of_exploration) / 8
                    else:
                        self.east -= 5 * math.sqrt(self.area_of_exploration) / 8
                elif rem == 3:
                    if self.robot_ID % 2 == 0:
                        self.east -= 5 * math.sqrt(self.area_of_exploration) / 8
                    else:
                        self.east += 5 * math.sqrt(self.area_of_exploration) / 8
                else:
                    self.north_rev -= self.dist_between_paths
            
            if j == self.number_of_cycles:
                self.last = 1

            lat, lon, _ = self.ned.ned2geodetic([self.east, self.north_rev, 0.0])

            mission_stepElem = etree.SubElement(missionElem, "mission_step")

            maneuverElem = etree.SubElement(mission_stepElem, "maneuver", type = "section")

            final_latitudeElem = etree.SubElement(maneuverElem, "final_latitude")
            final_latitudeElem.text = str(lat)

            final_longitudeElem = etree.SubElement(maneuverElem,"final_longitude")
            final_longitudeElem.text = str(lon)

            final_depthElem = etree.SubElement(maneuverElem,"final_depth")
            final_depthElem.text = "0"

            final_altitudeElem = etree.SubElement(maneuverElem,"final_altitude")
            final_altitudeElem.text = "1.0"

            heave_modeElem = etree.SubElement(maneuverElem,"heave_mode")
            heave_modeElem.text = "0"

            surge_velocityElem = etree.SubElement(maneuverElem,"surge_velocity")
            surge_velocityElem.text = "0.5"

            tolerance_xyElem = etree.SubElement(maneuverElem,"tolerance_xy")
            tolerance_xyElem.text = "1"

            no_altitude_goes_upElem = etree.SubElement(maneuverElem,"no_altitude_goes_up")
            no_altitude_goes_upElem.text = "true"

            last_sectionElem = etree.SubElement(maneuverElem,"last_section")
            last_sectionElem.text = str(self.last)

        outFile = open(self.mission_path, "wb")
        doc.write(outFile, pretty_print=True, xml_declaration=True, encoding='utf-8')
        print("...mission " + str(self.robot_ID) + " generated")

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
        mission = GenerateMission(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
