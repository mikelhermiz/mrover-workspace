import math
import numpy as np
from WayPoint import WayPoint
import conversions as conversions
import csv
import os
import json
import matplotlib.pyplot as plt


class Simulator():
    '''
        Path Generation based on a series of WayPoints given in a CSV
    '''


    def __init__(self):
        self.waypoints = []
        self.max_acceleration: float = .1
        self.max_velocity: float = 2
        self.dt: float = 5
        self.threshold: float = 1e-10

        # Hard coded file

        with open(os.path.join(os.getcwd(),"test1.csv"), "r") as points:
            for line in points.readlines():
                waypoint_info = line.split(",")
                self.waypoints.append(WayPoint(float(waypoint_info[0]), float(waypoint_info[1]), float(waypoint_info[2])))
        
        #read the reference lat and long from the file
        workspace_path = os.environ.get('OLDPWD')
        config_path = workspace_path + '/config/filter/config.json'
        with open(config_path) as config_json:
            config_dict = json.load(config_json)
            self.ref_long = config_dict['RefCoords']['long']
            self.ref_lat = config_dict['RefCoords']['lat']


    def path_gen(self):
        '''
        Main Path Generation algorithm
        Algorithm: Between each points given in the input calculate the distance traveled, velocity,
        acceleration, and bearing for each dt time. 

        '''
        output =[]
        time = 0
        all_the_time = []
        all_the_velocities = []
        all_the_accelerations = []
        all_the_lat = []
        all_the_long = []
        output.append([str(self.ref_lat), str(self.ref_long), str(0), str(0), str(0)])

        #wait for time_to_wait at the first point
        time_waited = 0

        while time_waited < self.waypoints[0].time_to_sit:
                output.append([str(self.ref_lat), str(self.ref_long), str(0), str(0), str(0)])
                time_waited += self.dt

        for waypoint in self.waypoints[:-1]:
            next_waypoint = self.waypoints[self.waypoints.index(waypoint)+1]
            total_distance = self.distance(waypoint, next_waypoint) 
            bearing = self.calculate_bearing(waypoint, next_waypoint)

            # Initialize Travel information
            time_traveled: float = self.dt
            distance_traveled: float = 0
            distance_needed_to_slow: float = 0
            distance_to_travel: float = total_distance
            velocity: float = 0
            acceleration: float = self.max_acceleration
            first_iteration = True

            while distance_to_travel >= self.threshold and (velocity >= self.max_acceleration*self.dt or first_iteration):
                distance_to_travel = total_distance - distance_traveled
                distance_needed_to_slow = (velocity)**2 / (2*self.max_acceleration)

                if distance_needed_to_slow >= distance_to_travel:
                    acceleration = -self.max_acceleration
            
                    if first_iteration:
                        first_iteration = False
                        velocity = math.sqrt(2*self.max_acceleration*distance_to_travel)

                    else:
                        velocity += acceleration * self.dt

                    distance_traveled += velocity*self.dt + 0.5*acceleration*(self.dt)**2

                else:
                    
                    if velocity == self.max_velocity:
                        acceleration = 0 

                    distance_traveled += velocity*self.dt + 0.5*acceleration*(self.dt)**2
                    velocity = min(self.max_velocity, self.max_acceleration * time_traveled)

                north_change = math.sin(bearing) * distance_traveled
                east_change = math.cos(bearing) * distance_traveled
                waypoint.north += north_change
                waypoint.east += east_change
                lat_info = self.calculate_lat(waypoint)
                long_info = self.calculate_long(waypoint, lat_info)
                bearing_deg = self.calculate_bearing_deg(bearing)
                output.append([lat_info, long_info, velocity, acceleration, bearing_deg])

                time_traveled += self.dt
                
                all_the_velocities.append(velocity)
                all_the_lat.append(lat_info)
                all_the_long.append(long_info)
                all_the_time.append(time)
                all_the_accelerations.append(acceleration)
                time = time + 1
            
            time_waited = 0
            while time_waited < next_waypoint.time_to_sit:
                output.append([lat_info, long_info, str(0), str(0), bearing_deg])
                time_waited += self.dt

            self.ref_lat = output[-1][0]
            self.ref_long = output[-1][1]

        
        self.outputPoints(output)

        plt.figure(1)
        ax1 = plt.subplot(311)
        ax1.set_title("Position")
        plt.plot(all_the_lat,all_the_long, 'ro')
        ax1.set_xlim([40,80])
        ax1.set_ylim([-90, -70])
        ax2 = plt.subplot(312)
        ax2.set_title("Velocity")
        plt.plot(all_the_time, all_the_velocities, 'ro')
        ax3 = plt.subplot(313)
        ax3.set_title("Acceleration")
        plt.plot(all_the_time, all_the_accelerations, 'ro')
        plt.show()
    

    def distance(self, start: WayPoint, end: WayPoint):
        '''
            Calculates distance between points
            @param WayPoint start: the begin point
            @param WayPoint end: the end point
            @return float: distance between points
        '''
        diff_north = math.fabs(end.north - start.north)
        diff_east = math.fabs(end.east - start.east)
        return math.hypot(diff_north, diff_east)

    def calculate_lat(self, point: WayPoint):
        '''
            Calculates the bearing between two points

            @param WayPoint point: a point relative to REFCOORDS
            @return float: degrees 0f latitude
        '''
        deg_lat = conversions.meters2lat(point.north, self.ref_lat)

        return deg_lat


    def calculate_long(self, point: WayPoint, lat_at_this_point):
        '''
            Calculates the bearing between two points

            @param WayPoint point: a point relative to REFCOORDS
            @return float: degrees of longitude
        '''
        deg_long = conversions.meters2long(point.east, lat_at_this_point, self.ref_long)

        return deg_long


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
            bearing = math.pi/2 + abs(math.atan(diff_north/diff_east))
        
        elif diff_north <= 0 and diff_east < 0:
            bearing = (3*math.pi)/2 - math.atan(diff_north/diff_east)

        elif diff_north >= 0 and diff_east < 0:
            bearing = (3*math.pi)/2 + abs(math.atan(diff_north/diff_east))
            
        return bearing

    def calculate_bearing_deg(self, bearing_angle):
        '''
            Changes angle from radians to degrees

            @param float bearing_angle: angle in radians
            @return float: angle in degrees
        '''
        return math.degrees(bearing_angle)

    def outputPoints(self, output):
        '''
            Outputs points to an output file.
            Hard coded output file.

        '''
        f = open('test1out.csv', 'w')
        f.write('lat_deg,   long_deg,   velocity,   acceleration,   bearing_deg')
        for x in output:
            f.write("\n")
            f.write(str(x[0]) + ",  " + str(x[1]) + ",  " + str(x[2]) + ",  " + str(x[3]) + ",  " + str(x[4]))
        f.close()

        


