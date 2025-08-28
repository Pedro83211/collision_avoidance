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
        self.mission_path = self.get_param('mission_path',"/home/pedro/catkin_ws/src/collision_avoidance/missions/mission_" + str(self.robot_ID) + ".xml")
        self.max_dist_between_paths = self.get_param('max_distance_between_paths', 5.0)
        side = math.sqrt(self.area_of_exploration)
        div_coefficient = 1
        east = 0
        north = 0
        last = 0

        # Set the number of cycles needed to cover the height of the square of exploration knowing that a cycle is
        # formed by 2 movements, except the first and the last cycles, being only one movement each

        if self.number_of_robots > 1:
            # Area of exploration of each AUV gets smaller in proportion to the total number of robots in the system
            if self.number_of_robots % 2 == 0:
                div_coefficient = self.number_of_robots / 2
            else:
                if self.robot_ID % 2 != 0:
                    div_coefficient = (self.number_of_robots - 1) / 2
                else:
                    div_coefficient = (self.number_of_robots + 1) / 2
    
        else:
            div_coefficient = 1

        counter = 0
        vertical_dist = self.max_dist_between_paths + 1
        while vertical_dist > self.max_dist_between_paths:
            vertical_moves = 4 * counter / 2 + 1
            vertical_dist = side / (div_coefficient * vertical_moves) # Number of movements needs to be divisible by 3 to cover the entire area
            counter += 1
        
        number_of_moves = 3 + 4*(counter - 1)
        
        # Set the initial point of each AUV depending on its ID
        if self.robot_ID % 2 != 0:
            east = -side / 2             
        else:
            east = side / 2

        north = side * (div_coefficient - 2 * math.floor(self.robot_ID / 2)) / (2 * div_coefficient)
        

        missionElem = etree.Element("mission")

        doc = etree.ElementTree(missionElem)

        number_of_moves +=  1 # To go from origin to start

        for i in range(number_of_moves):
            rem = i % 4 # There are 4 types of movements: to the right, down from the right, to the left, down from the left
            if i != 0:
                if rem == 1:
                    if self.robot_ID % 2 != 0:
                        east += 5 * side / 8
                    else:
                        east -= 5 * side / 8
                elif rem == 3:
                    if self.robot_ID % 2 != 0:
                        east -= 5 * side / 8
                    else:
                        east += 5 * side / 8
                else:
                    north -= vertical_dist
            
            if i == number_of_moves - 1:
                last = 1

            lat, lon, _ = self.ned.ned2geodetic([east, north, 0.0])

            mission_stepElem = etree.SubElement(missionElem, "mission_step")

            maneuverElem = etree.SubElement(mission_stepElem, "maneuver", type = "section")

            final_latitudeElem = etree.SubElement(maneuverElem, "final_latitude")
            final_latitudeElem.text = str(lat)

            final_longitudeElem = etree.SubElement(maneuverElem,"final_longitude")
            final_longitudeElem.text = str(lon)

            last_sectionElem = etree.SubElement(maneuverElem,"last_section")
            last_sectionElem.text = str(0) #++++++++++++++++++++++++++++++++++++++++++++++++str(last)

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
