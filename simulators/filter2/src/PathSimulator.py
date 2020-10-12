import math
import numpy as np
from WayPoint import WayPoint
import onboard.filter.src.conversions as conversions



class Simulator():
    '''
        Path Generation based on a series of WayPoints given in a CSV
    '''


    def __init__(self):
        self.waypoints = []
        self.max_acceleration = 0.5
        self.max_velocity = 2.0
        self.dt = 1e-3
        self.threshold = 1e-3
        # Hard coded file

        with open("test1.csv", "r") as points:
            for line in points.readlines():
                waypoint_info = line.split(",")
                self.waypoints.append(WayPoint(waypoint_info[0], waypoint_info[1], waypoint_info[2]))
    

    def path_gen(self):
        for waypoint in self.waypoints[:-1]:
            next_waypoint = waypoints[index(waypoint)+1]
            distance_to_travel = distance(waypoint, next_waypoint) 
            bearing = self.calculate_bearing(waypoint, next_waypoint)

            # Travel information
            velocity = 0
            acceleration = self.max_acceleration
            time_traveled = self.dt
            time_to_reach_max_velocity = self.max_velocity / self.max_acceleration
            
            while distance_to_travel > self.threshold:

                distance_traveled = 0
                distance_needed_to_slow = (velocity)**2 / (2*acceleration)
                if distance_to_travel <= distance_needed_to_slow:
                    acceleration = -self.max_acceleration
                    distance_traveled += velocity*self.dt + 0.5*acceleration*self.dt**2
                    velocity -= acceleration * self.dt

                else:
                    velocity = min(self.max_velocity, self.max_acceleration * time_traveled) 
                    #velocity += acceleration * time_traveled
                    distance_traveled = 1/2*acceleration*(time_traveled)**2

                time_traveled += self.dt
                distance_to_travel -= distance_traveled


    def distance(self, start: WayPoint, end: WayPoint):
        diff_north = math.fabs(end.north - start.north)
        diff_east = math.fabs(end.east - start.north)
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

        return (deg_lat, min_lat)


    def calculate_long(self, point: WayPoint):
        '''
            Calculates the bearing between two points

            @param WayPoint point: a point relative to REFCOORDS
            @return tuple: degrees and minutes of longitude
        '''
        deg_long = conversions.meters2long(point.east)
        min_long = conversions.decimal2min(deg_long%1)
        deg_long -= deg_long%1

        return (deg_long, min_long)


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
