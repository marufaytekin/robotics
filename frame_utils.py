import utm
import numpy


def global_to_local(global_position, global_home):
    # TODO: Get easting and northing of global_home
    (east_home, north_home, zone_number_home, zone_letter_home) = utm.from_latlon(global_home[1], global_home[0])
    # TODO: Get easting and northing of global_position
    (east, north, zone_number, zone_letter) = utm.from_latlon(global_position[1], global_position[0])
    # TODO: Create local_position from global and home positions
    local_position = numpy.array([north - north_home, east - east_home, -(global_position[2] - global_home[2])])

    return local_position


def local_to_global(local_position, global_home):
    # TODO: get easting, northing, zone letter and number of global_home
    (east, north, zone_number, zone_letter) = utm.from_latlon(global_home[1], global_home[0])
    # TODO: get (lat, lon) from local_position and converted global_home
    (latitude, longitude) = utm.to_latlon(east + local_position[1], north + local_position[0], zone_number, zone_letter)
    # TODO: Create global_position of (lat, lon, alt)
    global_position = numpy.array([longitude, latitude, -(local_position[2] - global_home[2])])

    return global_position


numpy.set_printoptions(precision=16)

geodetic_current_coordinates = [-122.079465, 37.393037, 30]
geodetic_home_coordinates = [-122.108432, 37.400154, 20]

local_coordinates_NED = global_to_local(geodetic_current_coordinates, geodetic_home_coordinates)

print(local_coordinates_NED)
assert (local_coordinates_NED == [-764.9642002619803, 2571.5906706879614, -10.]).all()


NED_coordinates =[25.21, 128.07, -30.]

# convert back to global coordinates
geodetic_current_coordinates = local_to_global(NED_coordinates, geodetic_home_coordinates)

print(geodetic_current_coordinates)
assert (geodetic_current_coordinates == [-122.10698241300805, 37.40037029050074, 50.0]).all()
