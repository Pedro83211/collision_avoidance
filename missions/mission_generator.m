clear variables
close all

wgs84 = wgs84Ellipsoid;
lat0 = 39.543330;
lon0 = 2.377940;
h0 = 0;

East = -20;
North = -20;

docNode = com.mathworks.xml.XMLUtils.createDocument('mission');
missionElem = docNode.getDocumentElement;

versionElem = docNode.createElement("version");
textNode = docNode.createTextNode("2.0");
versionElem.appendChild(textNode);

missionElem.appendChild(versionElem);

for i = 0:40
    if i ~= 0
        switch rem(i,4)
           case 1
                East = East + 30;
            case 3
                East = East - 30;
            otherwise
                North = North + 1;
        end
    end
    
    mission_stepElem = docNode.createElement("mission_step");

    maneuverElem = docNode.createElement("maneuver");
    maneuverElem.setAttribute('type','section');

    final_latitudeElem = docNode.createElement("final_latitude");
    final_longitudeElem = docNode.createElement("final_longitude");
    final_depthElem = docNode.createElement("final_depth");
    final_altitudeElem = docNode.createElement("final_altitude");
    heave_modeElem = docNode.createElement("heave_mode");
    surge_velocityElem = docNode.createElement("surge_velocity");
    tolerance_xyElem = docNode.createElement("tolerance_xy");
    no_altitude_goes_upElem = docNode.createElement("no_altitude_goes_up");

    [lat,lon,h] = ned2geodetic(North, East, 0,lat0, lon0, h0, wgs84);

    textNode = docNode.createTextNode(num2str(lat));
    final_latitudeElem.appendChild(textNode);
    
    textNode = docNode.createTextNode(num2str(lon));
    final_longitudeElem.appendChild(textNode);
    
    textNode = docNode.createTextNode("0");
    final_depthElem.appendChild(textNode);

    textNode = docNode.createTextNode(num2str("1.0"));
    final_altitudeElem.appendChild(textNode);

    textNode = docNode.createTextNode("0");
    heave_modeElem.appendChild(textNode);

    textNode = docNode.createTextNode("0.5");
    surge_velocityElem.appendChild(textNode);

    textNode = docNode.createTextNode("2.0");
    tolerance_xyElem.appendChild(textNode);

    textNode = docNode.createTextNode("true");
    no_altitude_goes_upElem.appendChild(textNode);

    maneuverElem.appendChild(final_latitudeElem);
    maneuverElem.appendChild(final_longitudeElem);
    maneuverElem.appendChild(final_depthElem);
    maneuverElem.appendChild(final_altitudeElem);
    maneuverElem.appendChild(heave_modeElem);
    maneuverElem.appendChild(surge_velocityElem);
    maneuverElem.appendChild(tolerance_xyElem);
    maneuverElem.appendChild(no_altitude_goes_upElem);
    
    mission_stepElem.appendChild(maneuverElem);

    missionElem.appendChild(mission_stepElem);
end

xmlwrite('mission.xml',docNode);

[lat,lon,h0] = ned2geodetic(20, 0, 0, lat0, lon0, h0, wgs84);

