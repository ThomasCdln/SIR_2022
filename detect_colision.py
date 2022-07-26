from threading import Thread
import time
import math


def get_distance_metres(lat1, lon1, lat2, lon2):
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = lat2 - lat1
    dlong = lon2 - lon1
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


class CollisionHandler(Thread):
    vehicles = []

    def __init__(self, *args):
        print("init collision handler")
        for i in args:
            self.vehicles.append(i)
        Thread.__init__(self)

    def run(self):
        # Insert threaded code
        while True:
            self.isCollision()
            time.sleep(0.1)

    def isCollision(self):
        for v1 in self.vehicles:
            for v2 in self.vehicles:
                if v1 != v2:
                    if get_distance_metres(v1.location.global_frame.lat, v1.location.global_frame.lon,
                                           v2.location.global_frame.lat, v2.location.global_frame.lon) < 1:
                        if v1.location.global_frame.alt - \
                                1 < v2.location.global_frame.alt < v1.location.global_frame.alt + 1:
                            print("....Collision....")
