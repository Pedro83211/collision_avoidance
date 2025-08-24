import rospy
import math
from cola2.utils.ned import NED
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
        self.dist_between_paths = self.get_param('distance_between_paths', 5.0)
        self.side = math.sqrt(self.area_of_exploration)
        self.cycle_coefficient = 1
        self.east = 0
        self.north = 0
        self.last = 0

        # Set the number of cycles needed to cover the height of the square of exploration knowing that a cycle is
        # formed by 2 movements, except the first and the last cycles, being only one movement each

        if self.number_of_robots > 1:
            # Area of exploration of each AUV gets smaller in proportion to the total number of robots in the system
            if self.number_of_robots % 2 == 0:
                self.cycle_coefficient = self.number_of_robots / 2
            else:
                if self.robot_ID % 2 != 0:
                    self.cycle_coefficient = (self.number_of_robots - 1) / 2
                else:
                    self.cycle_coefficient = (self.number_of_robots + 1) / 2
    
        else:
            self.cycle_coefficient = 1
        
        self.number_of_cycles = self.side / (2 * self.dist_between_paths * self.cycle_coefficient)
        
        # Set the initial point of each AUV depending on its ID
        if self.robot_ID % 2 != 0:
            self.east = -self.side / 2             
        else:
            self.east = self.side / 2

        self.north = self.side * (self.cycle_coefficient - 2 * math.floor(self.robot_ID / 2)) / (2 * self.cycle_coefficient)
        

        missionElem = etree.Element("mission")

        doc = etree.ElementTree(missionElem)

        number_of_moves = round(self.number_of_cycles * 4) + 1

        if (self.robot_ID == self.number_of_robots - 1):
            number_of_moves += 1

        for i in range(number_of_moves):
            rem = i % 4 # There are 4 types of movements: to the right, down from the right, to the left, down from the left
            if i != 0:
                if rem == 1:
                    if self.robot_ID % 2 != 0:
                        self.east += 5 * self.side / 8
                    else:
                        self.east -= 5 * self.side / 8
                elif rem == 3:
                    if self.robot_ID % 2 != 0:
                        self.east -= 5 * self.side / 8
                    else:
                        self.east += 5 * self.side / 8
                else:
                    self.north -= self.dist_between_paths

            lat, lon, _ = self.ned.ned2geodetic([self.east, self.north, 0.0])

            mission_stepElem = etree.SubElement(missionElem, "mission_step")

            maneuverElem = etree.SubElement(mission_stepElem, "maneuver", type = "section")

            final_latitudeElem = etree.SubElement(maneuverElem, "final_latitude")
            final_latitudeElem.text = str(lat)

            final_longitudeElem = etree.SubElement(maneuverElem,"final_longitude")
            final_longitudeElem.text = str(lon)

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
