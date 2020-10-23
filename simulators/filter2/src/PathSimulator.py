import math
import numpy as np
from WayPoint import WayPoint
import conversions as conversions
import csv
import os
# Bugs:
# - under the trapazoid case, velocity and acceleration behave as desired. under triangle case, velocity does not 
#   start slowing down soon enough such that it will end at zero by the time it goes far enough
# - lat/long don't update in the csv
# not implemented: 
# - lat and long relative to the reference coordinantes in config/filter/filter[RefCoords]
# - lat/long are still in degrees and minutes, need to make it just decimal degrees
# - doesn't wait the time that's desired between segments


class Simulator():
    '''
        Path Generation based on a series of WayPoints given in a CSV
    '''


    def __init__(self):
        self.waypoints = []
        self.max_acceleration: float = 0.5
        self.max_velocity: float = 1.0
        self.dt: float = 0.1
        self.threshold: float = 1e-10
        # Hard coded file

        with open(os.path.join(os.getcwd(),"test1.csv"), "r") as points:
            for line in points.readlines():
                waypoint_info = line.split(",")
                self.waypoints.append(WayPoint(float(waypoint_info[0]), float(waypoint_info[1]), float(waypoint_info[2])))
    

    def path_gen(self):
        output = []
        for waypoint in self.waypoints[:-1]:
            next_waypoint = self.waypoints[self.waypoints.index(waypoint)+1]
            total_distance = self.distance(waypoint, next_waypoint) 
            bearing = self.calculate_bearing(waypoint, next_waypoint)

            # Initialize Travel information
            time_traveled = 0
            distance_traveled = 0
            distance_needed_to_slow = 0
            distance_to_travel = total_distance
            velocity = 0
            acceleration = self.max_acceleration

            while distance_to_travel >= self.threshold:

                print("distance needed to slow:" + str(distance_needed_to_slow))
                if distance_needed_to_slow >= distance_to_travel:
                    acceleration = -self.max_acceleration
                    distance_traveled += velocity*self.dt + 0.5*acceleration*(self.dt)**2
                    print("distance traveled:" + str(distance_traveled))
                    velocity += acceleration * self.dt
                    north_change = math.sin(bearing) * distance_traveled
                    east_change = math.cos(bearing) * distance_traveled
                    waypoint.north += north_change
                    waypoint.east += east_change
                    print("evaluated ayaserpoqiahfsd")

                else:
                    velocity = min(self.max_velocity, self.max_acceleration * time_traveled)
                    if velocity == self.max_velocity:
                        acceleration = 0 
                    distance_traveled += velocity*self.dt + 0.5*acceleration*(self.dt)**2
                    north_change = math.sin(bearing) * distance_traveled
                    east_change = math.cos(bearing) * distance_traveled
                    waypoint.north += north_change
                    waypoint.east += east_change
                    print("distance traveled: " + str(distance_traveled))

                lat_info = self.calculate_lat(waypoint)
                long_info = self.calculate_long(waypoint, lat_info[0])
                bearing_deg = self.calculate_bearing_deg(bearing)
                output.append([lat_info[0], lat_info[1], long_info[0], long_info[1], velocity, acceleration, bearing_deg])

                time_traveled += self.dt
                distance_to_travel = total_distance - distance_traveled
                distance_needed_to_slow = (velocity)**2 / (2*self.max_acceleration)
                print("distance_to_travel:" + str(distance_to_travel))
        self.outputPoints(output)
    

    def distance(self, start: WayPoint, end: WayPoint):
        diff_north = math.fabs(end.north - start.north)
        diff_east = math.fabs(end.east - start.east)
        return math.hypot(diff_north, diff_east)

    def calculate_lat(self, point: WayPoint):
        '''
            Calculates the bearing between two points

            @param WayPoint point: a point relative to REFCOORDS
            @return tuple: degrees and minutes of latitude
        '''
        deg_lat = conversions.meters2lat(point.north)
        min_lat = conversions.decimal2min(deg_lat%1)
        deg_lat -= deg_lat%1

        return (deg_lat, min_lat[0])


    def calculate_long(self, point: WayPoint, lat):
        '''
            Calculates the bearing between two points

            @param WayPoint point: a point relative to REFCOORDS
            @return tuple: degrees and minutes of longitude
        '''
        deg_long = conversions.meters2long(point.east, lat)
        min_long = conversions.decimal2min(deg_long%1)
        deg_long -= deg_long%1

        return (deg_long, min_long[0])


    def calculate_bearing(self, start: WayPoint, end: WayPoint):
        '''
            Calculates the bearing between two points

            @param WayPoint start: starting waypoint
            @param WayPoint end: end waypoint
            @return float bearing: bearing between two WayPoints
        '''
        diff_north = end.north - start.north
        diff_east = end.east - start.east
        bearing = 0

        if diff_east == 0:
            if diff_north < 0:
                bearing = math.pi
            else:
                bearing = 0

        if diff_north >= 0 and diff_east > 0:
            bearing = math.pi/2 - math.atan(diff_north/diff_east)
        
        elif diff_north <= 0 and diff_east > 0:
            bearing = math.pi/2 + math.atan(diff_north/diff_east)
        
        elif diff_north <= 0 and diff_east < 0:
            bearing = (3*math.pi)/2 - math.atan(diff_north/diff_east)

        elif diff_north >= 0 and diff_east < 0:
            bearing = (3*math.pi)/2 + math.atan(diff_north/diff_east)
            
        return bearing

    def calculate_bearing_deg(self, bearing_angle):
        return math.degrees(bearing_angle)

    def outputPoints(self, output):
        '''
            Outputs points to an output file.
            Hard coded output file.
        '''
        f = open('test1out.csv', 'w')
        f.write('lat_deg,lat_min,long_deg,long_min,velocity,acceleration,bearing_deg\n')
        for x in output:
            f.write(str(x[0]) + "," + str(x[1]) + "," + str(x[2]) + "," + str(x[3]) + "," + str(x[4]) + "," + \
                str(x[5]) + "," + str(x[6]))
            f.write("\n")
        f.close()


