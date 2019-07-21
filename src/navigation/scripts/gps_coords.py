class GpsCoords():
    """
    Class to wrap a latitude/longitude fix
    The update() method can be used as a callback function for a ROS subscription
    of type sensor_msgs/NavSatFix
    
    This is a helper class, NOT a ROS node
    """

    def __init__(self, lat=0, lon=0):
        self.latitude = lat
        self.longitude = lon
    
    # handler for subscribed topic /fix
    def update(self, nav_sat_fix):
        self.latitude = nav_sat_fix.latitude
        self.longitude = nav_sat_fix.longitude
