

class WayPoint():
    '''
        Represents the North, East Coordinates of the rover's position 
        and how long to sit at that point.
    """
    '''

    def __init__(self, north, east, time_to_sit):
        self.north = north
        self.east = east
        self.time_to_sit = time_to_sit
    