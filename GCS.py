from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from detect_colision import CollisionHandler
from vehicle import Vehicle, getGcsPos, genPointsArray
import random
import os
from Random_point_generator import RandomPointGenerator
import math
import numpy as np
import threading
from math import cos, sin, tan, atan2, sqrt, radians, degrees, pi, log, fmod, ceil
import copy
from MAVProxy.modules.lib import mp_util


######## PARAMETERS ########
path2shareDic = "/mnt/hgfs/Connect/" # Path to the shared directory, depend on the configuration of your VM

# You shouldn't change that
radius_of_earth = 6378100.0  # in meters
######## PARAMETERS ########




def gps_distance(lat1, lon1, lat2, lon2):
    """distance between two points in meters"""
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    lon1 = radians(lon1)
    lon2 = radians(lon2)

    if abs(lat2 - lat1) < 1.0e-15:
        q = cos(lat1)
    else:
        q = (lat2 - lat1) / log(tan(lat2 / 2 + pi / 4) / tan(lat1 / 2 + pi / 4))
    d = sqrt((lat2 - lat1) ** 2 + q ** 2 * (lon2 - lon1) ** 2)
    return d * radius_of_earth


def gps_bearing(lat1, lon1, lat2, lon2):
    """return rhumb bearing between two points in degrees (range 0-360)"""
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    tc = -fmod(atan2(lon1 - lon2, log(tan(lat2 / 2 + pi / 4) / tan(lat1 / 2 + pi / 4))), 2 * pi)
    if tc < 0:
        tc += 2 * pi
    return degrees(tc)


def constrain(v, min_v, max_v):
    if v < min_v:
        v = min_v
    if v > max_v:
        v = max_v
    return v


def gps_newpos(lat, lon, bearing, dista):
    """extrapolate latitude/longitude given a heading and distance
    along rhumb line thanks to http://www.movable-type.co.uk/scripts/latlong.html
    """
    lat1 = constrain(radians(lat), -pi / 2 + 1.0e-15, pi / 2 - 1.0e-15)
    lon1 = radians(lon)
    tc = radians(-bearing)
    d = dista / radius_of_earth

    lat = lat1 + d * cos(tc)
    lat = constrain(lat, -pi / 2 + 1.0e-15, pi / 2 - 1.0e-15)
    if abs(lat - lat1) < 1.0e-15:
        q = cos(lat1)
    else:
        try:
            dphi = log(tan(lat / 2 + pi / 4) / tan(lat1 / 2 + pi / 4))
        except Exception:
            print(degrees(lat), degrees(lat1))
            raise
        q = (lat - lat1) / dphi
    d_lon = -d * sin(tc) / q
    lon = fmod(lon1 + d_lon + pi, 2 * pi) - pi
    return [degrees(lat), degrees(lon)]


def nbDroneMin(lat1, lng1, lat2, lng2, alt=6378100):
    # si le GCS est dans le coin de la piste :
    return max(1, ceil(gps_distance(lat1, lng1, lat2, lng2) / 100) - 1)


# si il est au centre: return ceil(max(1,distanceSol(lat1, lng1, lat2, lng2, alt )/100)/2)

def pathMaker(lat1, lng1, lat2, lng2, lat3, lng3, alt, scanWidth=2, altDrone=5, nbDrone=1):
    pointList = {}
    numEtape = 1
    # definir l'axe de la piste
    rwyForward = gps_bearing(lat1, lng1, lat2, lng2)
    rwyBackward = gps_bearing(lat2, lng2, lat1, lng1)
    rwyRight = gps_bearing(lat1, lng1, lat3, lng3)
    # definir les dimentions de la piste (repere cardinal relatif)
    distForward = gps_distance(lat1, lng1, lat2, lng2)
    distRight = gps_distance(lat1, lng1, lat3, lng3)
    # definir la pose de depart
    altDrone += alt
    posDepart = gps_newpos(lat1, lng1, (rwyForward + 45 + 360) % 360, scanWidth / 2 * sqrt(2))
    
    # actualiser la position du drone
    latDrone = posDepart[0]
    lngDrone = posDepart[1]
    # definir les points du trajet
    dir = 1  # current bearing : 1 rwyForward, 2 -rwyForward, 3 perpendiculaire
    currdistRight = 0
    while currdistRight < distRight - scanWidth:
        if dir == 1:  # vers le nord
            point = gps_newpos(latDrone, lngDrone, rwyForward, distForward - scanWidth)
            dir = 3
            prevDir = 1
            latDrone = point[0]
            lngDrone = point[1]
            point.append(altDrone)
            pointList[numEtape] = point
            numEtape += 1
        elif dir == 2:  # vers le sud
            point = gps_newpos(latDrone, lngDrone, rwyBackward, distForward - scanWidth)
            dir = 3
            prevDir = 2
            latDrone = point[0]
            lngDrone = point[1]
            point.append(altDrone)
            pointList[numEtape] = point
            numEtape += 1
        elif prevDir == 1:  # vers l'est depuis traj sud>nord
            point = gps_newpos(latDrone, lngDrone, rwyRight, scanWidth)
            dir = 2
            latDrone = point[0]
            lngDrone = point[1]
            point.append(altDrone)
            pointList[numEtape] = point
            numEtape += 1
            currdistRight += scanWidth
        else:  # vers l'est depuis traj nord>sud
            point = gps_newpos(latDrone, lngDrone, rwyRight, scanWidth)
            dir = 1
            latDrone = point[0]
            lngDrone = point[1]
            point.append(altDrone)
            pointList[numEtape] = point
            numEtape += 1
            currdistRight += scanWidth
    return pointList


def zones(lat1, lng1, lat2, lng2, lat3, lng3, alt=6378100):
    rwyForward = gps_bearing(lat1, lng1, lat2, lng2)
    rwyRight = gps_bearing(lat1, lng1, lat3, lng3)
    missionList = {}
    nbDrone = int(2 * nbDroneMin(lat2, lng2, lat3, lng3, alt))
    print(str(nbDrone) + " zones")
    widthZone = gps_distance(lat1, lng1, lat3, lng3)
    longTot = gps_distance(lat1, lng1, lat2, lng2)
    longZone = longTot / nbDrone
    oldLPoint = [lat1, lng1]
    oldRPoint = [lat3, lng3]
    newPoint = []
    newPoint = gps_newpos(lat1, lng1, rwyForward, longZone)
    for i in range(1, nbDrone+1):
        # print("left" + str(oldLPoint))
        # print("right" + str(oldRPoint))
        # print("new" + str(newPoint) + "\n")
        missionList[i] = pathMaker(oldLPoint[0], oldLPoint[1], newPoint[0], newPoint[1], oldRPoint[0], oldRPoint[1],
                                   alt)
        oldLPoint = newPoint
        oldRPoint = gps_newpos(oldLPoint[0], oldLPoint[1], rwyRight, widthZone)
        newPoint = gps_newpos(oldLPoint[0], oldLPoint[1], rwyForward, longZone)
    return missionList


def newProjectedPoI(lat1, lng1, lat2, lng2, lat3, lng3, alt=6378100):
    # define takeoff axes
    rwyForward = gps_bearing(lat1, lng1, lat2, lng2)
    rwyRight = gps_bearing(lat1, lng1, lat3, lng3)
    # definir les dimensions de la piste (repere cardinal relatif)
    distForward = gps_distance(lat1, lng1, lat2, lng2)
    distRight = gps_distance(lat1, lng1, lat3, lng3)
    # calcul des coordinates
    pos = []
    randDistFwd = random.uniform(100, distForward)
    randDistRight = random.uniform(0, distRight)
    pos.append(gps_newpos(lat1, lng1, rwyForward, randDistFwd)[0])
    pos.append(gps_newpos(lat1, lng1, rwyRight, randDistRight)[1])
    return pos


def poiGenerator(lat1, lng1, lat2, lng2, lat3, lng3, nb_points, alt=6378100):
    poiList = []
    for i in range(nb_points):
        poiList.append(newProjectedPoI(lat1, lng1, lat2, lng2, lat3, lng3, alt))
    return poiList


def writePOIFile(poi):
    """
        Method for writing a new connection address wich be use by Mission Planner

        Params:
            poi : Address to add to connectionList.txt
    """
    f = open(path2shareDic + "POI_1.txt", "a")  # Path to the shared directory, depend on the configuration of your VM
    for i in range(0, len(poi)):
        line = str(poi[i][0]) + "\t" + str(poi[i][1]) + "\t" + str(i) + "\n"
        print(line)
        f.write(line)
    f.close()

def resultsFile(results_time, results_bat):
    """
        Method for writing the results of the comparision of approaches 

        Params:
            results_time : dictionnary containning the time took by each approach
            results_bat : dictionnary containning the battery consomtion of each approach
    """
    f = open(path2shareDic + "results.txt", "a")  # Path to the shared directory, depend on the configuration of your VM
    for nDrones in results_time:
        res = results_time[nDrones]
        line = str(nDrones)+" : \n"
        f.write(line)
        for com in res:
            line = str(com)+" time : "+str(results_time[nDrones][com])+"\t"+"bat : "+str(results_bat[nDrones][com])+"\n"
            f.write(line)
    f.close()

def listToString(listed, separator=None):
    """
        Method to transform a list into a string

        Params :
            listed : the list that have to be transformed
            space : boolean wich specify if spaces have to be added between list elements

            return : a string containing all the elements of listed
    """
    string = ""
    for s in listed:
        string += str(s)
        if separator is not None:
            string += separator
    if separator is None:
        string += " "
    return string[:-1]

def distance(lat1, lon1, lat2, lon2):
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = lat2 - lat1
    dlong = lon2 - lon1
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

def get_distance_metres(lat, lon, lat2, lon2):
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    return mp_util.gps_distance(lat, lon, lat2, lon2)




class GCS:
    def __init__(self, args):
        # Thread
        self.ch = None
        # CONSTANTS :
        self.minLatIP = 0
        self.maxLatIP = 0
        self.minLonIP = 0
        self.maxLonIP = 0
        self.gcsLat = 0
        self.gcsLon = 0
        self.gcsAlt = 0
        #############

        # GLOBAL VARIABLES :
        self.vehicles = {}
        self.lastSendHalow = {}
        self.tHalow = None
        self.continu = True
        self.numberOfDetectedPoints = 0
        self.alignDrones = {}
        self.selectedDrones = []

        self.beginTime = None
        self.endTime = None
        self.beginBat = None
        self.endBat = None

        self.isHalow = False  # Set it to true to allow the halow communication on drones
        self.numberOfDronesInstances = 3
        self.isLogging = True  # set to true to allow all prints, False for only important ones

        self.finished = False
        self.detectingDrone = 0
        self.dif_bat = 0
        self.dif_time = 0
        ####################

        # ARGS management     arguments = {"lIP" : lIP , "WP" : WP, "isHalow" : isHalow, "nDrones" : nDrones, "isLogging" : isLogging}
        l_ip = args["lIP"] if "lIP" in args else None
        WP = args["WP"]
        self.numberOfDronesInstances = args["nDrones"]
        self.isHalow = args["isHalow"]
        self.isLogging = args["isLogging"]
        ho_me = args.get("home", None)
        if ho_me is not None:
            self.gcsLat = ho_me[0]
            self.gcsLon = ho_me[1]
            self.gcsAlt = ho_me[2]
            self.home = args["home"]

        # if a connection file exist, we delete it -> when drones instances are
        # created, they create the file and implement it
        if os.path.exists(path2shareDic+"connectionList.txt"):
            os.remove(path2shareDic+"connectionList.txt")
        # Not useful for now
        if os.path.exists(path2shareDic+"POI.txt"):
            os.remove(path2shareDic+"POI.txt")

        # create new set of interest points
        # rpg = RandomPointGenerator(-35.3634182, -35.3621443, 149.1651605, 149.1651776) # Have to set (min & max) latitude and longitude
        # lIP = rpg.getMultiplePoints(10) # Get random points of interest


        self.numberOfDetectedPoints = len(l_ip)

        # Create drones instances
        for i in range(1, self.numberOfDronesInstances + 1):
            self.vehicles[i] = Vehicle(
                {"nInstance": i, "lIP": l_ip, "logging": self.isLogging, "isHalow": self.isHalow, "path2shareDic": path2shareDic, "home":home, "WP":WP[i]})
            print("instance " + str(i) + " created")

        # Launch GCS halow messages handler
        self.tHalow = threading.Thread(target=self.handleHalowMsg, args=())
        self.tHalow.start()
        # Launch GCS wifi messages handler
        self.tWifi = threading.Thread(target=self.handleWifiMsg, args=())
        self.tWifi.start()

        # Initiate the collision handler
        if args.get("isCH"):
            self.ch = CollisionHandler(self.vehicles)
            self.ch.start()

        # set and start missions
        for vehi in self.vehicles:
            # Path to the shared directory, depend on the configuration of your VM
            self.vehicles[vehi].startMission()

        # Start the thread to look for end of missions
        self.tEndMission = threading.Thread(target=self.end_of_all, args=())
        self.tEndMission.start()

    def handleWifiMsg(self):
        """
            Method to handle classic wifi messages
            Run as a thread
        """
        while self.continu:
            for i in self.vehicles:
                msg = self.vehicles[i].getWifiMsg()
                if msg is not None:
                    if self.isLogging : print(
                        "receive wifi msg from :" +
                        str(i) +
                        " msg is : " +
                        str(msg))
                    if "video" in msg:
                        print("GCS receive the video from the drone")
                        sMsg = msg.split(":")
                        if len(sMsg) == 3:
                            plainSendingPath = sMsg[2] 
                            id_sending_path = plainSendingPath.split(";")
                            reverse_sending_path = id_sending_path[::-1]
                            print("id detecting drone : " + str(id_sending_path))
                            self.detectingDrone = int(id_sending_path[0])

                            if isHalow:
                                self.endTime = time.time()
                                self.endBat = self.vehicles[i].getBat()
                                for d in id_sending_path:
                                    if d != "0":
                                        self.sendHalowMsg(int(d), "START")
                            else:
                                msgToSend = "video:ok:" + \
                                    str(listToString(reverse_sending_path, ";"))
                                self.vehicles[i].wifiFromGcs = msgToSend

                        # Decrease the number of points to discover
                        self.numberOfDetectedPoints -= 1
                        # Resetting the current message
                        self.vehicles[i].wifiMsgGcs = None
                        print("There are " + str(self.numberOfDetectedPoints) + " points left")
            time.sleep(0.5)
        print("End of handleWifiMsg by GCS")





    def handleHalowMsg(self):
        """
            Method to handle Wi-Fi halow messages
            Run as a thread
        """
        while self.continu:
            for i in self.vehicles:
                msg = self.vehicles[i].halowMsg
                if not (i in self.lastSendHalow):  # for initialisation purpose only
                    self.lastSendHalow[i] = None
                if msg != self.lastSendHalow[i] and msg is not None:
                    print(
                        "receive wifi hallow msg from :" +
                        str(i) +
                        " msg is : " +
                        str(msg))
                    self.reset_halow_msg(i)
                    lMsg = msg.split(':')
                    if len(lMsg) == 2:
                        if lMsg[0] == "IP":
                            # A drone have detected an IP
                            sMsg = lMsg[1].split(";")
                            # Get the battery at the beginning of detection
                            self.beginTime = time.time()
                            self.beginBat = self.vehicles[i].getBat()
                            self.detectingDrone = i

                            self.handleAlignR(sMsg[0], sMsg[1], i)
                            
                        elif lMsg[0] == "ALIGN":
                            if lMsg[1] == "OK":
                                # The drone got to asked position
                                print("drone " + str(i) + " is aligned")
                        elif lMsg[0] == "VID":
                            if lMsg[1] == "OK":
                                print("Video have been sent")
                        elif lMsg[0] == "POS":
                            sMsg = lMsg[1].split(";")
                            if len(sMsg) == 3:
                                # The drone gives its position
                                print(
                                    "Drone " +
                                    str(i) +
                                    " position is : " +
                                    str(sMsg))
                            else:
                                print(
                                    "Drone " +
                                    str(i) +
                                    " doesn't have the right position message")
                        else:
                            print("Message doesn't have the right format")
                    else:
                        time.sleep(0.1)
        print("End of handleHalowMsg by GCS")


    def handleAlignR(self, a_lat, a_lon, r_id):
        """
            Method to handle the alignment for creating the multi hop network
        """
        time.sleep(1)
        print(
            "gcsLat, gcsLon, gcsAlt : " +
            str(self.gcsLat) +
            " " +
            str(self.gcsLon) +
            " " +
            str(self.gcsAlt))
        self.selectedDrones = [r_id]
        dronesPositions = self.getDronesPosHalow()
        points = genPointsArray(
            float(a_lat),
            float(a_lon),
            float(self.gcsAlt),
            float(self.gcsLat),
            float(self.gcsLon),
            float(self.gcsAlt),
            )
        print("Generated points")
        print(points)
        print(dronesPositions)
        orderOfAlign = []
        for p in points:
            point = points[p]
            rLat = point[0]
            rLon = point[1]
            bestDist = 10000
            bestId = 0
            for iD in dronesPositions:
                if iD not in self.selectedDrones:
                    dPos = dronesPositions[iD]
                    dist = distance(rLat, rLon, dPos[0], dPos[1])
                    if bestDist > dist:
                        bestDist = dist
                        bestId = iD
            self.selectedDrones.append(bestId)
            orderOfAlign.append(bestId)
            self.alignDrones[bestId] = [rLat, rLon]
        reverseOrderedAlign = orderOfAlign[::-1]

        # sending order to selected drones to align
        print("Align drones : " + str(self.alignDrones))
        for aD in self.alignDrones:
            rPos = self.alignDrones[aD]
            # ask drone to get to position
            self.sendHalowMsg(aD, "ALIGN:" + str(rPos[0]) + ";" + str(rPos[1]))

        # check if selected drones are at position

        for aD in self.alignDrones:
            arrived = False
            while not arrived:
                if self.vehicles[aD].halowMsg == "ALIGN:OK":
                    arrived = True
                time.sleep(0.01)
        if self.isLogging : print("All drones gets aligned")
        # sending the list of aligned drones to the requester drone
        msg = "ALIGNED:"
        for iD in orderOfAlign:
            msg += str(iD) + ";"
        # we send the list of aligned drones to the requester ( and drop the last
        # ;)
        self.sendHalowMsg(r_id, msg[:-1])


    def getDronesPosHalow(self, i_d = None):
        """
            Method that asks all drones their position

            Params:
                    iD : iD of a drone that we want to skip
                    return : a dictionnary containing position of all drones excepted iD if specified
        """
        pos = {}
        self.sendAllExceptOneHalow("POS", None)  # ask drones for their position
        # getting answers
        for i in range(1, len(self.vehicles) + 1):
            if i != i_d:
                msg = self.vehicles[i].halowMsg
                while msg is None or msg == "POS":
                    time.sleep(0.1)
                    msg = self.vehicles[i].halowMsg
                print("msg from " + str(i) + " is : " + str(msg))
                tmp = msg.split(":")
                if tmp[0] == "POS":
                    sPart = tmp[1]
                    tmp = sPart.split(";")
                    pos[i] = float(tmp[0]), float(tmp[1])
        return pos


    def sendAllExceptOneHalow(self, msg, excepted):
        """
            Method to send halow messages to all drones excepted one

            Params:
                    msg : the message to send
                    excepted : id of the drone excepted
        """
        print("sending : " + msg + " to all drones excepted : " + str(excepted))
        for i in range(1, len(self.vehicles) + 1):
            if i != excepted:
                self.sendHalowMsg(i, msg)


    def sendHalowMsg(self, i_d, msg):
        """
            Method to send halow message to one drone

            Params:
                    iD : ID of the drone that we want to send the message
                    msg : message to send to the drone
        """
        if self.isLogging : print("Halow msg send by GCS to "+str(i_d)+"is : "+str(msg))
        self.lastSendHalow[i_d] = msg
        self.vehicles[i_d].halowMsg = msg
        self.vehicles[i_d].isNewHalowMsg = True


    def reset_halow_msg(self, i_d):
        """
            Method to reset halow messages of a drone

            Params:
                    iD : id of the drone for wich we want to reset the messages
        """
        self.vehicles[i_d].halowMsg = None
        self.vehicles[i_d].isNewHalowMsg = False



    def end_of_all(self):
        """
            Method to close all at the end of all drones missions
        """
        end = False
        nOfDrones = len(self.vehicles)
        fDrones = 0
        # We check if all drones finished their missions
        time.sleep(nOfDrones)
        while self.continu and not end:
            for vehicle in self.vehicles.values():
                if vehicle.missionState:
                    fDrones = fDrones+1
                else :
                    fDrones = 0
                if fDrones == nOfDrones:
                    end = True
            fDrones = 0
            time.sleep(1)
        # We close the terminal instance containing the drones simulators and the mavlink simulators
        self.vehicles[1].closeWin()
        for veh in self.vehicles.values():
            # We stop all vehicle threads
            veh.close()
        # We stop all threads of this instance of GCS
        self.continu = False

        self.dif_bat = self.vehicles[self.detectingDrone].dif_bat
        self.dif_time = self.vehicles[self.detectingDrone].dif_time

        self.finished = True



if __name__ == '__main__':
    # Initialisation of table results
    tableOfResultsBat = {}
    tableOfResultsTime = {}

    if os.path.exists(path2shareDic+"results.txt"):
        os.remove(path2shareDic+"results.txt")

    # Number of Points of Interest 
    poiNumber = 1

    mission2 = zones(41.2930275, 2.0655853, 41.2934568, 2.0668057, 41.2924270, 2.0659500)
    mission4 = zones(41.2930275, 2.0655853, 41.29388, 2.0680529, 41.2924270, 2.0659500)
    mission8 = zones(41.2930275, 2.0655853, 41.2947546, 2.0703864, 41.2924270, 2.0659500)
    missions = {2: mission2, 4:mission4, 8:mission8}
    # List of Points of Interest 
    pois = {2:[[41.2934568, 2.0668057]], 4:[[41.29388, 2.0680529]], 8:[[41.2947546, 2.0703864]]}
    

    isHalow = True
    # Barcelona airport : 41.3061778,2.1050781,0.162556,353  
    home = [41.29274130, 2.06577170, 0.162556, 353]

    # listOfS = [2, 2, 4, 4, 8, 8]
    listOfS = [2, 2]

    for ind in listOfS:
        poi = pois[ind]
        arguments = {"isLogging" : True, "lIP" : poi[:] , "WP" : missions[ind], "isHalow" : isHalow, "nDrones" : ind, "home" : home}
        writePOIFile(pois[ind])
        v = GCS(arguments)
        # Waiting for missions to finish
        while not v.finished:
            time.sleep(2)
        # get the difference of battery level and time took
        # initialisation only :
        if tableOfResultsBat.get(ind) == None:
            tableOfResultsBat[ind] = {}
        if tableOfResultsTime.get(ind) == None:
            tableOfResultsTime[ind] = {}
        if isHalow:
            tableOfResultsBat[ind]["Halow"] = copy.copy(v.dif_bat)
            tableOfResultsTime[ind]["Halow"] = copy.copy(v.dif_time)
        else:
            tableOfResultsBat[ind]["Wi-Fi"] = copy.copy(v.dif_bat)
            tableOfResultsTime[ind]["Wi-Fi"] = copy.copy(v.dif_time)
        print('')
        print("######################################")
        print("battery levels : ")
        print(tableOfResultsBat)
        print("Time taken : ")
        print(tableOfResultsTime)
        print(v.dif_time)
        print(v.dif_bat)
        print("")
        # inversion of isHalow
        isHalow = not isHalow
        del v
        time.sleep(5)

    print("End of all simulations")
    print("######################################")
    print("battery levels : ")
    print(tableOfResultsBat)
    print("Time taken : ")
    print(tableOfResultsTime)
    resultsFile(tableOfResultsTime, tableOfResultsBat)

