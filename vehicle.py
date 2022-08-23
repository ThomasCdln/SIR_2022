# -*- coding: utf-8 -*-
from threading import Thread
import os
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import time
import subprocess
from math import pi, sqrt
from MAVProxy.modules.lib import mp_util
import logging
import heapq
import collections
import numpy as np
import random

###### GLOBALS ######
winTerm = 0  # ID of the terminal window where we launch commands
isTerm = False  # Boolean to know if there is an existing terminal window
lIP = [(0, 0)]  # list of interest point
wifiCom = {}  # embended dictionnary that contains messages send by wifi, first key : instance number of the sender, second key : instance number of the receiver
dronesPos = {}  # contain current drones instances positions | key : instance number
gcsLat = 0  # latitude of the GCS - setted at the takeoff point of the drone
gcsLon = 0  # longitude of the GCS - setted at the takeoff point of the drone
gcsAlt = 0  # altitude of the GCS - setted at the takeoff point of the drone
path2shareDic = "/mnt/hgfs/Connect/" # Path to the shared directory, depend on the configuration of your VM
###### GLOBALS ######


def getGcsPos():
    """
        Getter for gcs Latitude and Longitude

        Params:
            return gcsLat, gcsLon
    """
    global gcsLat, gcsLon, gcsAlt
    return gcsLat, gcsLon, gcsAlt


def get_distance_metres(lat, lon, lat2, lon2):
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    return mp_util.gps_distance(lat, lon, lat2, lon2)


def listToString(listed, separator=None):
    '''
        Method to transform a list into a string

        Params :
            listed : the list that have to be transformed
            space : boolean wich specify if spaces have to be added between list elements

        return : a string containing all the elements of listed
    '''
    string = ""
    for s in listed:
        string += str(s)
        if separator is not None:
            string += separator
    if separator is None:
        string += " "
    return string[:-1]


def genPoint(lat1, lon1, alt1, lat2, lon2, alt2, dist):
    '''
        Method to generate points on a line separed by a certain distance

        Params:
            lat1 : latitude of point 1
            lon1 : longitude of point 1
            alt1 : altitude of point 1
            lat2 : latitude of point 2
            lon2 : longitude of point 2
            alt2 : altitude of point 2
            dist : distance between generated points
    '''
    a = (lon1 - lon2) / (lat1 - lat2)
    b = lon1 - a * lat1
    x, y, l1, l2 = 0, 0, 0, 0
    if lat1 < lat2:
        l1 = lat1
        l2 = lat2
    else:
        l1 = lat2
        l2 = lat1
    for x in np.linspace(l1, l2, 100000):
        y = a * x + b
        d = get_distance_metres(lat1, lon1, x, y)
        if dist * 0.999 < d < dist:
            return x, y
    return None, None


def genPointsArray(lat1, lon1, alt1, lat2, lon2, alt2, dist=95):
    """
        Method wich generate points on a line between two points, spaced by dist

        Params:
            lat1 : latitude of the first point
            lon1 : longitude of the first point
            alt1 : altitude of the first point
            lat2 : latitude of the second point
            lon2 : longitude of the second point
            alt2 : altitude of the second point
            (dist) : distance between generated points (in metres)

            return : a dictionnary containing all generated points {'index':[lat, lon]}
    """
    begin = time.time()
    i = 0
    ui = {}
    newX, newY = genPoint(lat1, lon1, alt1, lat2, lon2, alt2, dist)
    ui[i] = newX, newY
    i += 1
    while lat1 > newX > lat2:
        x, y = genPoint(newX, newY, alt1, lat2, lon2, alt2, dist)
        if newX == x or x is None:
            break
        else:
            ui[i] = x, y
            i += 1
            newX = x
            newY = y
    return ui


def distance(lat1, lon1, alt1, lat2, lon2, alt2):
    '''
        Method to get the distance in meters between two points

        Params:
            lat1 : latitude of point 1
            lon1 : longitude of point 1
            alt1 : altitude of point 1
            lat2 : latitude of point 2
            lon2 : longitude of point 2
            alt2 : altitude of point 2

            return : distance in meters
    '''
    if alt1 is None:
        alt1 = 0
    if alt2 is None:
        alt2 = 0
    diffLat = abs(lat1 - lat2)
    diffLon = 6731000 * abs(lon1 - lon2)
    avgAlt = (alt1 + alt2) / 2
    distLat = avgAlt * (pi / 180) * diffLat
    distLon = avgAlt * (pi / 180) * diffLon
    distAlt = abs(alt1 - alt2)
    return sqrt(distAlt * distAlt + distLat * distLat + distLon * distLon)


def newTerminal():
    '''
        Method wich create a new terminal window and get it's ID
    '''
    global winTerm, isTerm
    os.system("lxterminal")
    time.sleep(0.5)
    winTerm = subprocess.check_output("xdotool getactivewindow", shell=True)
    winTerm = winTerm.replace("\n", "")
    isTerm = True


def runInTerminal(s):
    '''
        Method wich launch terminal commands such as the dronekit-sitl instance

        Params :
            s : the command to run as a list
    '''
    global winTerm, isTerm
    if not isTerm:
        newTerminal()
    else:
        os.system("xdotool windowactivate %s" % str(winTerm))
        os.system("xdotool key ctrl+Shift_L+t")
    os.system(
        "xdotool type --window " +
        winTerm +
        " '%s'" %
        listToString(
            s,
            " "))
    os.system("xdotool key --window " + winTerm + " Return")
    os.system("xdotool windowminimize --sync " + winTerm)


def genMavProxyCmd(instance):
    '''
        Method wich generate the command to launch mavproxy in terminal

        Params:
            intance : current dronekit-sitl instance number
            return : String with the IP address and port for connecting the GCS
    '''
    launchMavproxy = [
        "mavproxy.py",
        "--master",
        "tcp:127.0.0.1:5770",
        "--out",
        "udp:127.0.0.1:14551",
        "--out",
        "udp:10.0.2.2:14552"]
    tcpDrone = ["tcp:", "127.0.0.1", ":", "5770"]
    # change here the ip address to the router address
    outGCS = ["udp:", "192.168.235.1", ":", "14551"]
    outCmd = ["udp:", "127.0.0.1", ":", "14552"]
    GCS = ["udp:", "192.168.235.128:", ""]
    pTcp = 5760 + 10 * instance
    pOutGCS = 14550 + (instance * 2 - 1)
    pOutCmd = 14550 + (instance * 2)
    tcpDrone[3] = str(pTcp)
    outGCS[3] = str(pOutGCS)
    outCmd[3] = str(pOutCmd)
    GCS[2] = str(pOutGCS)
    launchMavproxy[2] = listToString(tcpDrone, None)
    launchMavproxy[4] = listToString(outGCS, None)
    launchMavproxy[6] = listToString(outCmd, None)
    writeConnectionFile(GCS)
    runInTerminal(launchMavproxy)
    return outCmd


def launchDroneKit(instance, startPoint=None):
    '''
        Method to start the drone kit siltl instance in a terminal

        Params :
            instance : instance number of the drone
            startPoint)  :
    '''
    launchDronekit = []
    if startPoint is None:
        launchDronekit = [
            "dronekit-sitl",
            "copter-3.3",
            "--instance",
            "instanceNumber"]
    else:
        # --home=-35.363261,149.165230,584,353
        # Barcelona airport : 41.3061778,2.1050781,0.162556,353  -- didn't
        # found how to get the fourth parameter for now
        launchDronekit = ["dronekit-sitl", "copter-3.3", "--instance", "instanceNumber", "--home=" + str(
            startPoint[0]) + "," + str(startPoint[1]) + "," + str(startPoint[2]) + "," + str(startPoint[3])]
    launchDronekit[3] = str(instance)
    runInTerminal(launchDronekit)


def writeConnectionFile(newAdd):
    '''
        Method for writing a new connection address wich be use by Mission Planner

        Params:
            newAdd : Address to add to connectionList.txt
    '''
    global path2shareDic
    if newAdd[0] == "udp:":
        newAdd[0] = "udp://"
    # Path to the shared directory, depend on the configuration of your VM
    f = open(path2shareDic+"connectionList.txt", "a")
    f.write(listToString(newAdd, None) + "\n")
    f.close()


def MPTFFtoDictio(FileName):
    '''
        Method which transform mission Planner waypoints to a dictionary containning those waypoints

        Prams:
            FileName : path to the mission planner generated waypoints file
    '''
    dico = {}
    altAboveSea = 0
    with open(FileName) as f:
        for i, line in enumerate(f):
            if i == 0:
                pass
            elif i == 1:
                linearray = line.split('\t')
                altAboveSea = float(linearray[10])
            else:
                linearray = line.split('\t')
                ln_index = int(linearray[0])
                ln_X = float(linearray[8])
                ln_Y = float(linearray[9])
                ln_Z = (float(linearray[10]) + altAboveSea)
                dico[ln_index] = [ln_X, ln_Y, ln_Z]
        return dico


def insertOrReplace(minHeap, element, weight):

    # Insert if does not exist
    if element not in [x[1] for x in minHeap]:
        heapq.heappush(minHeap, (weight, element))

    # Replace otherwise
    else:
        indexToUpdate = [x[1] for x in minHeap].index(element)
        minHeap[indexToUpdate] = (weight, element)
        heapq.heapify(minHeap)



###################################################################
class Vehicle():

    # Possible arguments :
    def __init__(self, args):
        '''
            Method called when we create a new instance of the class
        '''
        global lIP, path2shareDic
        # Instances
        self.vehicle = None          # dronekit vehicle instance, initialized in droneInstance()
        # Positions
        self.takeOffLat = None       # latitude at takeoff
        self.takeOffLon = None       # longitude at takeoff
        self.takeOffAlt = None       # alitude below sea level at takeoff
        self.curLat = 0              # current drone global latitude
        self.curLon = 0              # current drone global longitude
        self.curAlt = 0              # current drone global altitude
        self.stopPos = []            # Coordinates where the drone stopped
        self.wayPoints = {}          # index : [lat, lon, alt]
        # Flags
        # set to false to stop allow to stop command control and being able to
        # send another
        self.beg_bat = 0
        self.end_bat = 0
        self.dif_bat = 0
        self.beg_time = 0
        self.end_time = 0
        self.dif_time = 0
        self.isHold = False          # Flag that indicate if we want the drone to Hover
        self.isStop = False          # Flag to know if the drone is stopped
        self.missionState = False    # Flag to know if the mission is finished by the drone 
        # setted to true when we ask the drone to pause it's current mission
        # and go to a position
        self.isOnMission = False
        # Boolean value setted to True if there is a new halow message to read
        self.isNewHalowMsg = False
        self.isHalow = False         # True if Halow-WiFi can be used on this drone instance
        self.isHandleWifi = True
        self.logging = False         # Set it to True if you want to have all the logs
        self.running = True          # True if the instance still running, else false
        # Messages
        self.halowMsg = None         # msg from the Wifi-Halow
        self.wifiMsgGcs = None       # wifiMsgToGCS
        self.wifiFromGcs = None      # wifiMsgFromGCS
        # dictionnarie containning the last received wifi msg from other drones
        self.wifiMsgOld = {}
        # dictionnarie containning the roads to communicate with each drone
        self.msgRoads = {}
        # Threads
        self.tMission = None         # thread to manage missions
        self.tAat = Thread()         # thread to manage arm and takeoff
        self.tIpD = None             # thread to discover points of interest
        # End of declarations
        # 
        self.nInstance = args["nInstance"] if "nInstance" in args else None
        lIP = args["lIP"] if "lIP" in args else [(0, 0)]
        startPoint = args["home"] if "home" in args else None
        self.logging = args["logging"] if "logging" in args else False
        tkOffAlt = args["takeoffAltitude"] if "takeoffAltitude" in args else 20
        self.isHalow = args["isHalow"] if "isHalow" in args else False
        path2shareDic = args["path2shareDic"] if "path2shareDic" in args else path2shareDic
        self.wayPoints = args["WP"] if "WP" in args else None
        if not isinstance(self.nInstance, int):
            raise Exception()
        # trying to hide the logs from dronekit but seems not working
        if not self.logging:
            logging.getLogger('dronekit').addHandler(logging.NullHandler())
            logging.getLogger('autopilot').addHandler(logging.NullHandler())
        if self.logging:
            print("init vehicle number : " + str(self.nInstance))
        # we launch dronekit-sitl
        launchDroneKit(self.nInstance, startPoint)
        # we launch mavproxy and get the connexion address
        self.connectionAddr = genMavProxyCmd(self.nInstance)
        # we create a vehicle instance using the connexion address we just got
        self.tDroneInstance = Thread(
            target=self.droneInstance, args=(
                self.connectionAddr,))
        self.tDroneInstance.start()
        # we launch the thread to handle wifi communication
        self.tHndlW = Thread(target=self.handleReceiveWifi)
        self.tHndlW.start()
        # if the drone has an halow interface, we launch the thread to handle
        # halow communication
        if self.isHalow:
            self.tHndlH = Thread(target=self.handleReceiveHalow)
            self.tHndlH.start()
        # we create the thread that will handle the mission
        self.tMission = Thread(target=self.handleWayPoints, args=())
        # Needed because it's a threaded class
        # We launch the thread that will handle the discovering of points of
        # interest
        self.tIpD = Thread(target=self.iPDiscovering)
        self.tIpD.start()
        # We launch the thread to handle arm and takeoff of the drone
        self.tAat = Thread(target=self.aat, args=(tkOffAlt,))
        self.tAat.start()
        if self.logging:
            print("End of initialisation of vehicle class instance " +
                  str(self.nInstance))

    def droneInstance(self, connectAdd):
        '''
            Initiate the dronekit connection with the simulated drone

            Params:
                connectAdd : the connection IP and port as a List
        '''
        global gcsLat, gcsLon, gcsAlt
        connectAdd[0] = "udpin:"
        connectAdd[1] = "0.0.0.0"
        connectString = listToString(connectAdd, None)
        self.vehicle = connect(connectString, wait_ready=True)
        self.takeOffLat = self.vehicle.location.global_frame.lat
        self.takeOffLon = self.vehicle.location.global_frame.lon
        self.takeOffAlt = self.vehicle.location.global_frame.alt
        gcsLat = self.takeOffLat
        gcsLon = self.takeOffLon
        gcsAlt = self.takeOffAlt
        self.cmdToDrone = self.vehicle.commands

    #################################### Setters & Getters ###################
    def getBat(self):
        '''
            Getter for the battery level of the vehicle
        '''
        return int(self.vehicle.battery.level)

    def getPos(self):
        '''
            Getter for the localisation of the vehicle
        '''
        return self.curLat, self.curLon, self.curAlt

    def getVehicle(self):
        '''
            Getter for the dronekit vehicle instance
        '''
        self.tDroneInstance.join()
        return self.vehicle

    def setInterestPoint(self, listIP):
        '''
            Setter for the Interest Point list
        '''
        global lIP
        lIP = listIP

    def setWayPointsFromFile(self, filePath):
        '''
            Method to set waypoints from a file

            Params:
                filePath : path to waypoints file
        '''
        self.wayPoints = MPTFFtoDictio(filePath)

    def setWayPoints(self, wayPoints):
        '''
            Method to set waypoints

            Params:
                wayPoints : dictionnary containning waypoints with format 'index' : [lat, lon, alt]
        '''
        self.wayPoints = wayPoints

    def iPDiscovering(self):
        '''
            Method wich will be run when we start the threaded class
        '''
        global dronesPos, lIP
        if self.logging:
            print("run thread of instance " + str(self.nInstance))
        # checking if vehicle is well connected
        self.tDroneInstance.join()
        # checking if vehicle had takeoff
        self.tAat.join()
        while self.running:
            # Update the position of the drone
            self.curLat = self.vehicle.location.global_frame.lat
            self.curLon = self.vehicle.location.global_frame.lon
            self.curAlt = self.vehicle.location.global_frame.alt
            # Update the drone position for all other instances of drones (this class)
            dronesPos[self.nInstance] = [self.curLat, self.curLon, self.curAlt]
            # Check for points of interest by checking for a distance lower than 2 meters between the POI and the drone + checking if the drone follow its mission or if its alligning 
            for p in lIP:
                if (get_distance_metres(
                        p[0], p[1], self.curLat, self.curLon) < 3) and self.isOnMission:
                    print("Point detected by drone " + str(self.nInstance))
                    self.beg_bat = self.getBat()
                    self.beg_time = time.time()
                    dist = get_distance_metres(
                        self.curLat, self.curLon, self.takeOffLat, self.takeOffLon)
                    if self.logging:
                        print("distance between GCS and IP : " + str(dist))
                    # if a POI is detected, we handle it and remove it from the list of POI
                    self.handleIP()
                    lIP.remove(p)
            # We do this check all 0.5 seconds
            time.sleep(0.5)
        if self.logging : print(" iPDiscovering of "+str(self.nInstance)+" is closed")

    def handleIP(self):
        '''
            Method called when a Interst Point is detected
        '''
        # First we stop the drone over the interest point
        self.stopDrone()
        # After we check if the GCS is near enought to receive the video of the
        # interest point
        if get_distance_metres(
                self.takeOffLat, self.takeOffLon, self.curLat, self.curLon) < 100:
            if self.logging:
                print("The GCS is near enought to receive directly the video")
            self.wifiMsgGcs = "video:" + str(self.nInstance)+";0"
        else:
            # If the drone have an halow interface
            if self.isHalow:
                # send a message to GCS to inform it that an interest point is
                # detected
                if self.logging:
                    print("Inform GCS that there is an interest point")
                # message structure : 'IP,self.lat; self.lon'
                self.halowMsg = "IP:" + \
                    str(self.curLat) + ";" + str(self.curLon)
            # if the drone doesn't have an halow interface 
            else:
                # handle this POI with a custom method using only wifi
                self.handleIPWifi(self.curLat, self.curLon)

    def aat(self, aTargetAltitude):
        '''
            Arm and takeoff the drone

            Params :
                aTargetAltitude : targeted altitude
        '''
        # wainting for connection finished
        self.tDroneInstance.join()

        if self.logging:
            print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while self.running and not self.vehicle.is_armable:
            if self.logging:
                print(
                    " Waiting for vehicle %i to initialise..." %
                    self.nInstance)
            time.sleep(1)

        if self.logging:
            print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while self.running and not self.vehicle.armed:
            if self.logging:
                print(" Waiting for arming...")
            time.sleep(1)
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True

        if self.logging:
            print("Taking off!")
        # Take off to target altitude
        self.vehicle.simple_takeoff(aTargetAltitude)

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while self.running and not (
                self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95):
            if self.logging:
                print(
                    " Altitude: ",
                    self.vehicle.location.global_relative_frame.alt)
            time.sleep(1)
        if self.logging:
            print("Reached target altitude")

    def getDistanceBtwId(self, id1, id2):
        global dronesPos
        return get_distance_metres(dronesPos[int(id1)][0], dronesPos[int(
            id1)][1], dronesPos[int(id2)][0], dronesPos[int(id2)][1])



    #################################### WIFI COM ############################
    # Scheme : we first establish the mapping of the network, then we can send
    # to anyone using it (prevent broadcast wich is a pain in the ass)

    def sendWifi(self, iD, msg):
        '''
            Method to send a wifi msg to a drone iD

            Params:
                    msg : message to send
        '''
        global wifiCom, dronesPos
        if self.msgRoads != {}:
            if len(self.msgRoads[int(iD)]) < 2:
                msg = msg + ":" + \
                    str(self.msgRoads[int(iD)][0]) + ";" + str(iD)
            else:
                msg = msg + ":" + \
                    listToString(self.msgRoads[int(iD)], ";") + ";" + str(iD)
                iD = self.msgRoads[int(iD)][1]

        # If the specified ID is 0, it means that it's a message for the GCS
        if int(iD) == 0:
            if self.logging : print(str(self.nInstance) + " send msg to GCS : " + str(msg))
            self.wifiMsgGcs = msg
        # If it's not a message for the GCS
        else:
            listOfIntermediates = []
            dicMsg = {}
            # We initialise the sending dictionary if it doesn't exist
            if not str(self.nInstance) in wifiCom.keys():
                wifiCom[str(self.nInstance)] = {}
            # If the drone is under 100m away
            if (self.getDistanceBtwId(int(iD), int(self.nInstance)) < 115):
                # If there is already a non read message we append the message we want to send to the current one
                if (wifiCom.get(str(self.nInstance)) is not None) and (wifiCom[str(self.nInstance)].get(
                        str(iD)) is not None) and (wifiCom[str(self.nInstance)][str(iD)] != "None"):
                    # We append it by separating them by a |
                    wifiCom[str(self.nInstance)][str(iD)] += "|" + msg
                else:
                    # If there is no message unread, we just put the message in the sending buffer
                    wifiCom[str(self.nInstance)][str(iD)] = msg
                if self.logging:
                    print("drone " + str(self.nInstance) +
                          " directly send : " + msg)
            else:
                # If the drone wich we want to send the message is too far and if logging is activated for this drone, we print it 
                if self.logging:
                    print("drone " + str(iD) + " is too far ")
                    print("drone " + str(iD) + " is at "+str(self.getDistanceBtwId(int(iD), int(self.nInstance))))

    def getWifiMsg(self):
        '''
             Method for the GCS to get wifi messages from drone
        '''
        # Before executing this method, we first need the drone to be well initialized 
        self.tDroneInstance.join()
        # We assume that the drone doesn't send messages to the GCS until it has take off, so we wait for it to take of
        self.tAat.join()
        # Getting the distance between the GCS (assume that it's position is at the takeoff point of the drone)
        dist = get_distance_metres(
            self.takeOffLat,
            self.takeOffLon,
            self.curLat,
            self.curLon)
        # If the distance between the GCS and the drone is under 100m, the GCS calling this method is able to read the data sended by the drone 
        if dist < 100:
            return self.wifiMsgGcs
        else:
            return None

    def handleReceiveWifi(self):
        '''
            method to search for new wifi messages from other drones and from the GCS
        '''
        global wifiCom
        # First we wait for the drone instance to be initialized 
        self.tDroneInstance.join()
        # We initialise the comparator of old message wich is used to know wich messages has been already received ( to prevent the drone to interprate the same message in loop (may not be usefull))
        oldGcsVal = None
        # We check for new messages while self.running is to true 
        while self.running:
            # we check for new message only if the drone is able to handle wifi commands (if its not the drone that discoverd a new POI, in this case there is a custom method)
            if self.isHandleWifi:
                # check for message from GCS
                if self.wifiFromGcs != oldGcsVal and self.wifiFromGcs is not None:
                    oldGcsVal = self.wifiFromGcs
                    # If there is a new message, we handle it 
                    self.handleWifiCommand(str(self.wifiFromGcs))
                    # Just checking if the drone has an halow interface, in this case there is a custom message, it should be modified to be handled like other messages but didn't had time 
                    if self.isHalow:
                        if self.wifiFromGcs == "VID:OK":
                            # The GCS inform the drone that it can start again
                            # its mission
                            self.startMission()
                    # Resetting the message buffer from the GCS because the message has been handled 
                    self.wifiFromGcs = None
                # check for message from other drones
                msgs = self.getNewWifiMsg()
                # If there is a queue of messages, we handle them one after the other 
                for msg in msgs:
                    self.handleWifiCommand(msg)
            # Wait for 0.1 second before reading the sending buffer of all drones again 
            time.sleep(0.1)
        if self.logging : print(" handleReceiveWifi of "+str(self.nInstance)+" is closed")

    def getNewWifiMsg(self):
        '''
            Method to get new message(s) from WiFi

            Params:
                    Return : list of messages
        '''
        global wifiCom
        # initialise the message to return in case there are no message to handle by the drone 
        sMsg = []
        # Searching among all sended messages of all drones instances 
        for di in wifiCom.keys():
            # Getting the sending buffer of all drones instances 
            curDic = wifiCom.get(di)
            # If there is a sending buffer 
            if curDic is not None:
                # We try to get a message for the current drone 
                msg = curDic.get(str(self.nInstance))
                # Checking if there is a message for the current drone and checking if we are near enough to get it (less than 100 meters)
                if (di != str(self.nInstance)) and (msg is not None) and (
                        msg != "None") and (self.getDistanceBtwId(di, self.nInstance) < 115):
                    # If there is (a) readable message(s), try to split it by the separator indicating that there is a queue of messages 
                    sMsg = msg.split('|')
                    # Resetting the current sending message buffer 
                    curDic[str(self.nInstance)] = None
                    # Return the message(s) that have to be handled 
                    return sMsg
        return sMsg

    def handleWifiCommand(self, msg):
        '''
            Method to handle commands sends by WiFi
            Kinds of commands and relevant parameters :
                video:listofnextreceivers

            Params:
                    msg : message to handle
        '''
        # If logging is enabled, we print the message that has to be handled 
        if self.logging:
            print("wifi message to handle by " +
                  str(self.nInstance) + " : " + msg)
        # We initialize the list of reachables drones in case the message command is "map"
        reachables = []
        # We split the message to extract all parts of it (command:parameters:sendingPath)
        sMsg = msg.split(":")
        command = sMsg[0]
        parameters = sMsg[1]
        plainSendingPath = sMsg[2]
        # We prepare the sending path
        idSendingPath = plainSendingPath.split(";")
        # and the reverse sending path if the drone has to handle the command : it's the last id in the sending path 
        reverseSendingPath = idSendingPath[::-1]
        # We prepare the next drone index to send it the message in the case this drone has to handle the command and respond to it 
        nextIndex = reverseSendingPath.index(str(self.nInstance)) + 1
        # First we split the message
        sMsg = msg.split(":")
        # check if we have to forward it
        if idSendingPath[-1] == str(self.nInstance):
            # we do not forward it
            if sMsg[0] == "map":
                # the sender ask for the near drones of the current drone
                # first we stop
                self.stopDrone()
                # Then we get the list of near enough drones (under 100 meters)
                for droneId in dronesPos:
                    if (droneId != self.nInstance) and (get_distance_metres(
                            dronesPos[droneId][0], dronesPos[droneId][1], self.curLat, self.curLon) < 100):
                        reachables.append(droneId)
                # Then we answer with the list we just got 
                self.sendWifi(
                    reverseSendingPath[nextIndex],
                    command +
                    ":" +
                    listToString(
                        reachables,
                        ";") +
                    ":" +
                    listToString(
                        reverseSendingPath,
                        ";"))
            if sMsg[0] == "start":
                # The drone is asked to start it's mission
                self.startMission()
            if sMsg[0] == "align":
                # The drone is asked to align
                tempS = parameters.split(";")
                if self.logging:
                    print("The drone " +
                          str(self.nInstance) +
                          " is asked to align")
                self.handleAlign(float(tempS[0]), float(tempS[1]))
                # When the drone is align to the required position we answer to the resquesting that the drone is aligned 
                self.sendWifi(
                    reverseSendingPath[nextIndex],
                    "aligned::" +
                    listToString(
                        reverseSendingPath,
                        ";"))
            if sMsg[0] == "pos":
                # The drone is asked about its position
                if parameters == "":
                    # We answer the current position of the drone 
                    self.sendWifi(reverseSendingPath[nextIndex], command + ":" + str(
                        self.curLat) + ";" + str(self.curLon) + ":" + listToString(reverseSendingPath, ";"))
        else:
            # we forward the message
            if self.logging:
                print("drone " + str(self.nInstance) +
                      " forward the message : " + msg)
            nextIndex = idSendingPath.index(str(self.nInstance)) + 1
            self.sendWifi(idSendingPath[nextIndex], msg)

    def handleIPWifi(self, ipLat, ipLon):
        '''
            Method to handle POI when the halow interface is not enable

            Params:
                    ipLat : latitude of the point of interest
                    ipLon : longitude of the point of interest
        '''
        # Saving current time and battery level of the drone 
        beginTime = time.time()
        beginBat = self.vehicle.battery.level

        nAlign = 0
        # First we ask get the network mapping
        self.getWifiMapping()
        # generate the list of points that have to be used
        neededPos = genPointsArray(
            ipLat,
            ipLon,
            self.curAlt,
            self.takeOffLat,
            self.takeOffLon,
            self.takeOffAlt)
        # We get all drones positions
        dPos = self.getPosWifi()
        # Then choose wich drone goes where
        assignations = collections.OrderedDict()
        for point in neededPos:
            tempBestId = 0
            tempBestDist = 1000
            for iD in dPos:
                curPos = dPos[iD]
                dist = get_distance_metres(
                    neededPos[point][0],
                    neededPos[point][1],
                    curPos[0],
                    curPos[1])
                if dist < tempBestDist and iD not in assignations.keys():
                    tempBestId = iD
                    tempBestDist = dist
            assignations[tempBestId] = str(
                neededPos[point][0]) + ";" + str(neededPos[point][1])
        if self.logging:
            print(" points assignations : " + str(assignations))
        # Now we send the drones to their required positions
        for i in assignations:
            self.sendWifi(int(i), "align:" + assignations[i])
  
        # Check if all drones are aligned
        self.isHandleWifi = False
        if self.logging : print("Stopping the wifi handling thread")
        # We check if they're all aligned
        if self.logging:
            print("Check for drones alignement")
        while nAlign < len(assignations):
            # check for message from other drones
            msgs = self.getNewWifiMsg()
            for msg in msgs:
                sMsg = msg.split(":")
                if (sMsg[0] == "aligned"):
                    nAlign += 1
        # Start the receiving thread
        if self.logging:
            print("All drones are aligned")

        # Send the video to the GCS
        self.msgRoads[0] = [int(self.nInstance)]
        for k in assignations.keys():
            self.msgRoads[0].append(int(k))
        self.sendWifi(0, "video:")

        # Wainting to receive the confirmation of the receiving of the video 
        isVidRec = False
        while not isVidRec:
            msgs = self.getNewWifiMsg()
            for msg in msgs:
                sMsg = msg.split(":")
                if sMsg[0] == "video" and sMsg[1] == "ok":
                    isVidRec = True
        
        endTime = time.time()
        endBat = self.vehicle.battery.level

        print("")
        print(str(self.nInstance) +
              " Total time for handle the IP without Halow : " +
              str(endTime -
                  beginTime))
        print(str(self.nInstance) +
              " Total battery consumtion in percentage for handle the IP without Halow : " +
              str(int(beginBat) -
                  int(endBat)))
        print("")

        # Now we send back the drones used to their initial position
        for drones in assignations.keys():
            print("Initial position of drone " +
                  str(drones) + " is " + str(dPos[drones]))
            self.sendWifi(int(drones), "align:" +
                          str(dPos[drones][0]) + ";" + str(dPos[drones][1]))

        nAlign = 0
        # We check if they're all aligned
        if self.logging:
            print("Check for drones alignement")
        while nAlign < len(assignations):
            # check for message from other drones
            msgs = self.getNewWifiMsg()
            for msg in msgs:
                sMsg = msg.split(":")
                if (sMsg[0] == "aligned"):
                    nAlign += 1
        # Start the receiving thread
        if self.logging:
            print("All drones are aligned")

        # Then we restart all missions
        for drones in self.msgRoads:
            # 0 is the GCS, it doesn't have a mission
            if int(drones) != 0:
                self.sendWifi(int(drones), "start:")

        # IP is handle, the drone can continu its mission
        self.startMission()

        # Resetting the map because it's not valid anymore
        self.msgRoads = {}
        # Allow wifi reception again
        self.isHandleWifi = True

        

    def getPosWifi(self):
        '''
            Method to get the position of all drones by wifi multi hop network

            Params:
                    return : dictionnary of other drones positions
        '''
        nPos = 0
        posDic = {}
        # Stop the receiving thread
        self.isHandleWifi = False
        # We ask for all drones positions by sending a broadcast message
        if self.logging:
            print("Drone " + str(self.nInstance) +
                  " ask for all drones position")
        for i in self.msgRoads:
            self.sendWifi(int(i), "pos:")

        if self.logging:
            print("Messages send to all drone to get their positions")
        # Then we check for answer
        while nPos < len(self.msgRoads):
            # check for message from other drones
            msgs = self.getNewWifiMsg()
            for msg in msgs:
                sMsg = msg.split(":")
                if (sMsg[0] == "pos") and (";" in sMsg[1]):
                    pos = sMsg[1].split(";")
                    roads = sMsg[2].split(";")
                    posDic[roads[0]] = [float(pos[0]), float(pos[1])]
                    nPos += 1
        # Start the receiving thread
        self.isHandleWifi = True

        if self.logging:
            print("positions : " + str(posDic))
        return posDic

    def getWifiMapping(self):
        '''
            Method used to establish the network mapping
        '''
        global dronesPos
        nearEnoughtDrones = []
        discoverdDrones = [str(self.nInstance)]
        alreadyGetPos = [str(self.nInstance)]
        self.tDroneInstance.join()
        self.tAat.join()
        curNetMap = {}
        curNetMap[int(self.nInstance)] = []
        # We establish the list of near enought drones
        for droneId in dronesPos:
            if (droneId != self.nInstance) and (get_distance_metres(
                    dronesPos[droneId][0], dronesPos[droneId][1], self.curLat, self.curLon) < 100):
                curNetMap[int(self.nInstance)].append(int(droneId))
                curNetMap[int(droneId)] = [int(self.nInstance)]
                nearEnoughtDrones.append(int(droneId))
        if self.logging:
            print("initial mapping : " + str(curNetMap))
        # Then we get their nearest drones
        for near in curNetMap[int(self.nInstance)]:
            self.sendWifi(
                near,
                "map::" +
                listToString(
                    self.defineRoad(
                        str(near),
                        curNetMap),
                    ";"))
            discoverdDrones.append(str(near))

        # Stop the receiving thread
        self.isHandleWifi = False
        # Then start scanning for all drones
        while len(alreadyGetPos) != len(discoverdDrones):
            for nearDrone in nearEnoughtDrones:
                msgs = self.getNewWifiMsg()
                for msg in msgs:
                    sMsg = msg.split(":")
                    if sMsg[1] != "":
                        answer = sMsg[1].split(';')
                        road = sMsg[2].split(';')
                        alreadyGetPos.append(road[0])
                        for iD in answer:
                            if iD not in discoverdDrones:
                                if int(road[0]) not in curNetMap.keys():
                                    curNetMap[int(road[0])] = [int(iD)]
                                elif int(iD) not in curNetMap[int(road[0])]:
                                    curNetMap[int(road[0])].append(int(iD))
                                if int(iD) not in curNetMap.keys():
                                    curNetMap[int(iD)] = [int(road[0])]
                                elif int(road[0]) not in curNetMap[int(iD)]:
                                    curNetMap[int(iD)].append(int(road[0]))
                                self.sendWifi(
                                    int(nearDrone),
                                    "map::" +
                                    listToString(
                                        self.defineRoad(
                                            str(iD),
                                            curNetMap),
                                        ";"))
                                discoverdDrones.append(iD)
            time.sleep(1)
        # Start the receiving thread
        self.isHandleWifi = True
        if self.logging : print("dicoverd drones : " + str(discoverdDrones))
        # then the mapping should be complete
        if self.logging:
            print("complete map : " + str(curNetMap))
        # Then we define the roads :
        self.defineOptimizedRoads(curNetMap, discoverdDrones)

    def defineRoad(self, receiver, curNetMap):
        '''
            Define a road between two drones using the Dijkstra algorythm

            Params:
                curNetMap : dictionnary of drones neighbours 
                receiver : drone to wich we want to send the message at the end

                return : list of drones id that preceed the sender and allow it to reach the receiver
        '''
        # We will use the Dijkstra algorythm so we need to identify drones as nodes starting from 0 and have contiguous numbers
        # To do so, we use the index of a list containing each drone instance number as the node number
        # This list is named "decoder"
        decoder = []
        # We generate the "decoder" list 
        for i in curNetMap.keys():
            decoder.append(str(i))
        # We apply the dijkstra algorythm and get the list of predecessors for each drones 
        predecessors = self.dijkstraAlgo(curNetMap, decoder)
        # Then we get the "encoded" number of the receiver
        tempPred = predecessors[int(decoder.index(str(receiver)))]
        # We initialize the list of predecessors needed to reach the "receiver"
        tempList = [int(decoder.index(str(receiver)))]
        # We generate the full list of predecessors to reach the "receiver"
        if tempPred is not None:
            tempList.append(int(tempPred))
            tempPred2 = predecessors[int(tempPred)]
            while tempPred2 is not None:
                if tempPred2 is not None:
                    tempList.append(int(tempPred2))
                tempPred2 = predecessors[int(tempPred2)]
            tempList = tempList[::-1]
        road = []
        # We "decode" it to get the true drones instance number 
        for el in tempList :
            road.append(int(decoder[el]))
        if self.logging:
            print(self.msgRoads)
        # We return the full roads containing predecessors drones IDs 
        return road
        

    def defineOptimizedRoads(self, curNetMap, decoder):
        '''
            Method to find the different roads to communicate with each drone

            Params:
                curNetMap : dictionnary of drones neighbours 
                receiver : drone to wich we want to send the message at the end
        '''
        # This method do quite the same thing than the one before, but for all receivers
        predecessors = self.dijkstraAlgo(curNetMap, decoder)
        tempDic = {}
        tempList = []
        for index in range(len(predecessors)):
            tempPred = predecessors[index]
            if tempPred is not None:
                tempList.append(int(tempPred))
                tempPred2 = predecessors[int(tempPred)]
                while tempPred2 is not None:
                    if tempPred2 is not None:
                        tempList.append(int(tempPred2))
                    tempPred2 = predecessors[int(tempPred2)]
                tempDic[index] = tempList[::-1]
                del tempList[:]
        # decode
        tempDecode = {}
        for key, item in tempDic.items():
            tempDecode[int(decoder[int(key)])] = []
            for it in item:
                tempDecode[int(decoder[int(key)])].append(
                    int(decoder[int(it)]))
        self.msgRoads = tempDecode
        if self.logging:
            print(self.msgRoads)

    def dijkstraAlgo (self, curNetMap, decoder):
        '''
            Implementation of the dijkstra algorythm

            Params:
                curNetMap : Dictionnary of neighbours between drones
                decoder : List use as coder and decoder for dijkstra nodes
        '''
        # first we gen a matrix data structure and pass it to graph from
        # curNetMap
        if self.logging:
            print("defineOptimizedRoads")
        # encode the dictionnary to be used by the algorythm
        tempEncode = {}
        for key, item in curNetMap.items():
            tempEncode[decoder.index(str(key))] = []
            for it in item:
                tempEncode[decoder.index(str(key))].append(
                    int(decoder.index(str(it))))

        curNetMap = tempEncode

        od = collections.OrderedDict(sorted(curNetMap.items()))

        graph = []
        tempGraph = []
        tempTuple = []
        for iD in od:
            for near in od[iD]:
                tempTuple.append(int(near))
                tempTuple.append(1)
                tempGraph.append(tuple(tempTuple[:]))
                del tempTuple[:]
            graph.append(tempGraph[:])
            del tempGraph[:]

        sourceNode = decoder.index(str(self.nInstance))

        distances = [float("inf") for i in range(len(graph))]
        minHeap = [(0, sourceNode)]
        distances[sourceNode] = 0
        predecessors = [None for i in range(len(graph))]

        while len(minHeap) != 0:
            # We extract the closest node from the heap
            (closestNodeDistance, closestNode) = heapq.heappop(minHeap)
            # We update the distance to the neighbors of this node
            for (neighbor, weight) in graph[closestNode]:
                neighborDistance = closestNodeDistance + weight
                if neighborDistance < distances[neighbor]:
                    insertOrReplace(minHeap, neighbor, neighborDistance)
                    distances[neighbor] = neighborDistance
                    predecessors[neighbor] = closestNode
        return predecessors
      


    #################################### HALOW Com ###########################

    def handleReceiveHalow(self):
        '''
            Method searching for new halow messages
        '''
        # Waiting for the drone to be initialized and has take off
        self.tDroneInstance.join()
        self.tAat.join()
        while self.running:
            if self.isNewHalowMsg:
                self.isNewHalowMsg = False
                self.handleHalowMsg(self.halowMsg)
            time.sleep(0.1)
        if self.logging : print(" handleReceiveHalow of "+str(self.nInstance)+" is closed")

    def handleHalowMsg(self, msg):
        '''
            Method to react to Halow messages sended to this drone

            Params:
                msg : halow msg send to the drone
        '''
        if self.logging:
            print("Halow message receive by drone " +
                  str(self.nInstance) + " is " + str(msg))
        lMsg = msg.split(':')
        if self.logging:
            print("splited msg give : " + str(lMsg))
        if len(lMsg) == 1:
            if msg == "STOP":
                # stop all drone movements
                self.stopDrone()
                if self.logging:
                    print("drone " + str(self.nInstance) + " stopped")
            elif "START" in msg:
                # start drone's movements
                self.startMission()
            elif msg == "POS":
                # answer the curent position of the drone
                self.halowMsg = "POS:" + \
                    str(self.curLat) + ";" + \
                    str(self.curLon) + ";" + str(self.curAlt)
            else:
                if self.logging:
                    print("Message doesn't have the right format")
        elif len(lMsg) == 2:
            if lMsg[0] == "ALIGN":
                # The drone is asked to be aligned by going to given point
                self.stopDrone()
                sMsg = lMsg[1].split(";")
                self.handleAlign(sMsg[0], sMsg[1])
                self.halowMsg = "ALIGN:OK"
            elif lMsg[0] == "ALIGNED":
                # List of the aligned drones to know to wich the video
                # transmition have to be done
                sMsg = lMsg[1].split(";")
                first = sMsg[0]
                tempMsg = "video::" + \
                    str(self.nInstance) + ";" + ';'.join(sMsg) + ";" + str(0)
                self.sendWifi(int(first), tempMsg)
            elif lMsg[0] == "VID":
                if lMsg[1] == "OK":
                    if self.logging:
                        print("Video have been sent")
            else:
                if self.logging:
                    print("Message doesn't have the right format")
        else:
            time.sleep(0.1)

    #################################### Drone mvmts #########################

    def startMission(self):
        '''
            Method to start the mission
        '''
        # First we wait for the drone to initialise and takeoff
        self.tDroneInstance.join()
        self.tAat.join()
        # We check if a mission have been provided to the drone
        if len(self.wayPoints) < 1:
            print("You have to import a mission first")
        # If the mission already started we just resume it
        elif self.tMission.is_alive():
            self.end_bat = self.getBat()
            self.end_time = time.time()
            self.dif_bat = self.beg_bat - self.end_bat 
            self.dif_time = self.end_time - self.beg_time
            if not self.isOnMission:
                if self.logging:
                    print("Drone " +
                          str(self.nInstance) +
                          " continue the mission")
                try:
                    self.sendCmdwControl(
                        self.stopPos[0], self.stopPos[1], self.stopPos[2])
                except e:
                    if self.logging:
                        print(e)
                self.isOnMission = True
                self.isStop = False
        # If the mission were not started, we start it
        else:
            if self.logging:
                print(str(self.nInstance) + " launch mission")
            self.isOnMission = True
            self.isStop = False
            self.tMission.start()

    def stopDrone(self):
        '''
            Method to stop the drone movements
        '''
        if self.logging:
            print("stop drone " + str(self.nInstance))
        # We stop the current mission
        self.isOnMission = False
        self.isStop = True  # to stop the current command
        # Wait for the drone to exit it's check in sendCmdWCtrl()
        time.sleep(1)
        # Save the position where the drone stopped
        self.stopPos = [self.curLat, self.curLon, self.curAlt]
        self.isStop = False  # to allow again to send new commands
        # Send the drone to it's position just to be shure it doesn't go to an
        # anwanted position
        self.vehicle.simple_goto(
            LocationGlobal(
                self.curLat,
                self.curLon,
                self.curAlt),
            airspeed=10)
        self.t_hold_pos(self.curLat, self.curLon, self.curAlt)
        


    def hold_pos(self, lat, lon, alt):
        '''
            Method to counter the auto disarm feature
        '''
        beg_time = time.time()
        while self.isHold:
            if (time.time()-beg_time) > 2:
                beg_time = time.time()
                randLat = random.uniform(lat, lat+0.00001)
                randLon = random.uniform(lon, lat+0.00001)
                self.vehicle.simple_goto(LocationGlobal(randLat, lon, self.curAlt))
            else:
                time.sleep(0.5)
                
    def t_hold_pos(self, lat, lon, alt):
        '''
            Method to launch the thread of position holding
        '''
        t_hold = Thread(target=self.hold_pos, args=(lat, lon, alt,))
        self.isHold = True
        t_hold.start()

    def sendCmdwControl(self, lat, lon, alt, speed=30):
        '''
            Method to send the drone to a certain point + wait to be there

            Params:
                lat : latitude of the destination point
                lon : longitude of the destination point
                alt : altitude of the destination point
                speed : (optional) set the drone speed, default is 20 m/s
        '''
        # You should not use this method
        self.tDroneInstance.join()
        self.tAat.join()
        self.isHold = False
        if self.logging:
            print("drone " + str(self.nInstance) + " goto : " +
                  str(lat) + " " + str(lon) + " " + str(alt))
        self.vehicle.simple_goto(
            LocationGlobal(
                lat,
                lon,
                self.curAlt),
            airspeed=speed)
        while not self.isStop and (get_distance_metres(
                lat, lon, self.curLat, self.curLon) > 1.2):
            dist = get_distance_metres(lat, lon, self.curLat, self.curLon)
            self.vehicle.simple_goto(
                LocationGlobal(
                    lat,
                    lon,
                    self.curAlt),
                airspeed=speed)
            time.sleep(0.5)

    def handleWayPoints(self):
        '''
            Method to run the mission given to the drone
        '''
        # You should not use this method
        i = 1
        maxId = len(self.wayPoints)
        if self.logging:
            print(" Nb commandes : " + str(maxId))
        while i <= maxId and self.running:
            if self.isOnMission:
                tempCmd = self.wayPoints[i]
                lat = tempCmd[0]
                lon = tempCmd[1]
                alt = tempCmd[2]
                self.sendCmdwControl(lat, lon, alt)
                if not self.isStop:  # if we exit sendCmdwControl because isStop, we dont want to go to another command as we want to continue the actual task when the drone is not stop anymore
                    i += 1
        print("Mission finished by drone " + str(self.nInstance))
        self.vehicle.mode = VehicleMode("RTL")
        self.missionState = True
        self.running = False

    def handleAlign(self, lat, lon, alt="600"):
        '''
            Method to stop the current mission and send the drone to a point

            Params:
                lat : latitude of the destination point
                lon : longitude of the destination point
                alt : altitude of the destination point
        '''
        # we tell it where to go
        if self.logging:
            print("Drone : " + str(self.nInstance) + "  aligning")
        self.sendCmdwControl(float(lat), float(lon), float(alt))
        # Once it gets there, we inform the operator with a terminal print, and
        # others drones with a wifi message
        if self.logging:
            print("Drone : " + str(self.nInstance) + "  aligned !")
        # we stop the drone
        self.stopDrone()
    #################################### Close the instance ##################

    def close(self):
        '''
            Method for clean closing the class
        '''
        self.running = False
        self.tDroneInstance.join()
        self.tAat.join()
        try:
            self.vehicle.close()
        finally:
            print('closed ' + str(self.nInstance))

    def closeWin(self):
        '''
            Method for closing the window terminal of dronekit-sitl and mavproxy
        '''
        global winTerm, isTerm
        os.system("xdotool windowclose %s" % winTerm)
        isTerm = False


###################################################################
if __name__ == '__main__':
    # --home=-35.363261,149.165230,584,353
    ui1 = Vehicle({"nInstance": 1, "logging": True})
    ui2 = Vehicle({"nInstance": 2, "logging": False})
    time.sleep(40)

    ui1.getWifiMapping()
