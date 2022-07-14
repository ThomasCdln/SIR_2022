# -*- coding: utf-8 -*-
from threading import Thread
import os
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
# from dronekit_sitl import SITL 
import time
import subprocess
from math import pi, sqrt
from MAVProxy.modules.lib import mp_util
import logging


###### GLOBALS ######
winTerm = 0 # ID of the terminal window where we launch commands
isTerm = False # Boolean to know if there is an existing terminal window
lIP = [(0,0)] # list of interest point 
wifiCom = {} # # embended dictionnary that contains messages send by wifi, first key : instance number of the sender, second key : instance number of the receiver 
dronesPos = {} # contain current drones instances positions | key : instance number
gcsLat = 0 # latitude of the GCS - setted at the takeoff point of the drone
gcsLon = 0 # longitude of the GCS - setted at the takeoff point of the drone
gcsAlt = 0 # altitude of the GCS - setted at the takeoff point of the drone
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


def listToString(listed, space):
    '''
        Method to transform a list into a string

        Params :
            listed : the list that have to be transformed
            space : boolean wich specify if spaces have to be added between list elements

        return : a string containing all the elements of listed 
    '''
    string = ""
    for s in listed:
        string+= s
        if space :
            string+= " "
    return string


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
    a = (lon1-lon2)/(lat1-lat2)
    b = lon1 - a * lat1
    x, y, l1, l2 = 0, 0, 0, 0
    if lat1<lat2:
        l1 = lat1
        l2 = lat2
    else :
        l1 = lat2
        l2 = lat1
    for x in np.linspace(l1, l2, 100000):
        y = a*x+b
        d = distance(lat1, lon1, x, y)
        if dist*0.999<d<dist:
            return x, y
    return None, None


def genPointsArray(lat1, lon1, alt1, lat2, lon2, alt2, dist=100):
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
    newX,newY = genPoint(lat1, lon1, alt1, lat2, lon2, alt2, dist)
    ui[i] = newX,newY
    i+=1
    while lat1>newX>lat2:
        x,y = genPoint(newX, newY, alt1, lat2, lon2, alt2, dist)
        if newX == x or x==None:
            break
        else:
            ui[i] = x,y
            i+=1
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
    if alt1 == None:
        alt1 = 0
    if alt2 == None:
        alt2 = 0
    diffLat = abs(lat1-lat2)
    diffLon = 6731000*abs(lon1-lon2)
    avgAlt = (alt1+alt2)/2
    distLat = avgAlt*(pi/180)*diffLat
    distLon = avgAlt*(pi/180)*diffLon
    distAlt = abs(alt1-alt2)
    return sqrt(distAlt*distAlt + distLat*distLat + distLon*distLon)

    
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
    if not isTerm :
        newTerminal()
    else:
        os.system("xdotool windowactivate %s"%str(winTerm))
        os.system("xdotool key ctrl+Shift_L+t")
    os.system("xdotool type --window "+ winTerm + " '%s'"%listToString(s, True))
    os.system("xdotool key --window " + winTerm + " Return")
    os.system("xdotool windowminimize --sync "+ winTerm)    
    

def genMavProxyCmd(instance):
    '''
        Method wich generate the command to launch mavproxy in terminal

        Params:
            intance : current dronekit-sitl instance number
            return : String with the IP address and port for connecting the GCS
    '''
    launchMavproxy = ["mavproxy.py", "--master", "tcp:127.0.0.1:5770", "--out", "udp:127.0.0.1:14551", "--out", "udp:10.0.2.2:14552"]
    tcpDrone = ["tcp:", "127.0.0.1", ":", "5770"]
    outGCS = ["udp:", "192.168.235.1", ":", "14551"] # change here the ip address to the router address
    outCmd = ["udp:", "127.0.0.1", ":", "14552"]
    GCS = ["udp:","192.168.235.128:", ""]
    pTcp = 5760 + 10*instance
    pOutGCS = 14550 + (instance*2-1)
    pOutCmd = 14550 + (instance*2)
    tcpDrone[3] = str(pTcp)
    outGCS[3] = str(pOutGCS)
    outCmd[3] = str(pOutCmd)
    GCS[2] = str(pOutGCS)
    launchMavproxy[2] = listToString(tcpDrone, False)
    launchMavproxy[4] = listToString(outGCS, False)
    launchMavproxy[6] = listToString(outCmd, False)
    writeConnectionFile(GCS)
    runInTerminal(launchMavproxy)
    return outCmd


def launchDroneKit(instance, startPoint = None):
    '''
        Method to start the drone kit siltl instance in a terminal

        Params :
            instance : instance number of the drone
            startPoint)  :
    '''
    launchDronekit = []
    if startPoint == None :
        launchDronekit = ["dronekit-sitl", "copter-3.3", "--instance", "instanceNumber"]
    else:
        # --home=-35.363261,149.165230,584,353
        # Barcelona airport : 41.3061778,2.1050781,0.162556,353  -- didn't found how to get the fourth parameter for now
        launchDronekit = ["dronekit-sitl", "copter-3.3", "--instance", "instanceNumber", "--home="+ str(startPoint[0])+","+str(startPoint[1])+","+ str(startPoint[2])+","+ str(startPoint[3])]
    launchDronekit[3] = str(instance)
    runInTerminal(launchDronekit)


def writeConnectionFile(newAdd):
    '''
        Method for writing a new connection address wich be use by Mission Planner

        Params:
            newAdd : Address to add to connectionList.txt
    '''
    if newAdd[0]=="udp:":
        newAdd[0]="udp://"
    f = open("/mnt/hgfs/Connect/connectionList.txt", "a") # Path to the shared directory, depend on the configuration of your VM
    f.write(listToString(newAdd, False) + "\n")
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
            if i==0:
                pass
            elif i==1:
                linearray=line.split('\t')
                altAboveSea = float(linearray[10])
            else:
                linearray=line.split('\t')
		ln_index=int(linearray[0])
		ln_X=float(linearray[8])
		ln_Y=float(linearray[9])
		ln_Z=(float(linearray[10]) + altAboveSea)
		dico[ln_index] = [ln_X, ln_Y, ln_Z]
        return dico
    




###################################################################
class Vehicle():
    # Instances
    vehicle = None          # dronekit vehicle instance, initialized in droneInstance()
    cmdToDrone = None       # command instance used for sending commands to drone
    # Positions
    takeOffLat = None       # latitude at takeoff
    takeOffLon = None       # longitude at takeoff
    takeOffAlt = None       # alitude below sea level at takeoff
    curLat = 0              # current drone global latitude
    curLon = 0              # current drone global longitude
    curAlt = 0              # current drone global altitude
    stopPos = []            # Coordinates where the drone stopped
    wayPoints = {}          # index : [lat, lon, alt]
    # Flags
    isStop = False          # set to false to stop allow to stop command control and being able to send another
    isOnMission = False     # setted to true when we ask the drone to pause it's current mission and go to a position
    isNewHalowMsg = False   # Boolean value setted to True if there is a new halow message to read
    isHalow = False         # True if Halow-WiFi can be used on this drone instance
    logging = False         # Set it to True if you want to have all the logs
    running = True          # True if the instance still running, else false
    # Messages
    halowMsg = None         # msg from the Wifi-Halow 
    wifiMsgGcs = None       # wifiMsgToGCS
    wifiMsgOld = {}         # dictionnarie containning the last received wifi msg from other drones
    # Threads
    tMission = None         # thread to manage missions
    tAat = Thread()             # thread to manage arm and takeoff
    tIpD = None             # thread to discover points of interest

    

    # Possible arguments : 
    def __init__(self, args):
        '''
            Method called when we create a new instance of the class
        '''
        global lIP
        self.nInstance = args["nInstance"] if "nInstance" in args else None
        lIP = args["lIP"] if "lIP" in args else [(0,0)]
        startPoint = args["home"] if "home" in args else None
        self.logging = args["logging"] if "logging" in args else False
        tkOffAlt = args["takeoffAltitude"] if "takeoffAltitude" in args else 20
        if type(self.nInstance)!=int : raise Exception()
        # trying to hide the logs from dronekit but seems not working
        if not self.logging:
            logging.getLogger('dronekit').addHandler(logging.NullHandler())
            logging.getLogger('autopilot').addHandler(logging.NullHandler())
        if self.logging : print("init vehicle number : " + str(self.nInstance))
        # we launch dronekit-sitl
        launchDroneKit(self.nInstance, startPoint)
        # we launch mavproxy and get the connexion address 
        self.connectionAddr = genMavProxyCmd(self.nInstance)
        # we create a vehicle instance using the connexion address we just got
        self.tDroneInstance = Thread(target=self.droneInstance, args=(self.connectionAddr,))
        self.tDroneInstance.start()
        # we launch the thread to handle wifi communication
        self.tHndlW = Thread(target=self.handleReceiveWifi)
        self.tHndlW.start()
        # if the drone has an halow interface, we launch the thread to handle halow communication
        if self.isHalow:
            self.tHndlH = Thread(target=self.handleReceiveHalow)
            self.tHndlH.start()
        # we create the thread that will handle the mission
        self.tMission = Thread(target=self.handleWayPoints, args=())
        # Needed because it's a threaded class
        # We launch the thread that will handle the discovering of points of interest
        self.tIpD = Thread(target=self.iPDiscovering)
        self.tIpD
        # We launch the thread to handle arm and takeoff of the drone
        self.tAat = Thread(target=self.aat, args=(tkOffAlt,))
        self.tAat.start()


    
    def droneInstance(self, connectAdd):
        '''
            Initiate the dronekit connection with the simulated drone

            Params:
                connectAdd : the connection IP and port as a List 
        '''
        global gcsLat, gcsLon, gcsAlt
        connectAdd[0]="udpin:"
        connectAdd[1]="0.0.0.0"
        connectString = listToString(connectAdd, False)
        self.vehicle = connect(connectString, wait_ready=True)
        self.takeOffLat = self.vehicle.location.global_frame.lat
        self.takeOffLon = self.vehicle.location.global_frame.lon
        self.takeOffAlt = self.vehicle.location.global_frame.alt
        gcsLat = self.takeOffLat
        gcsLon = self.takeOffLon
        gcsAlt = self.takeOffAlt
        self.cmdToDrone = self.vehicle.commands





    #################################### Setters & Getters ####################################
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
        if self.logging : print("run thread of instance " + str(self.nInstance))
        # checking if vehicle is well connected
        self.tDroneInstance.join()
        # checking if vehicle had takeoff
        self.tAat.join()
        while self.running:
            self.curLat = self.vehicle.location.global_frame.lat
            self.curLon = self.vehicle.location.global_frame.lon
            self.curAlt = self.vehicle.location.global_frame.alt
            dronesPos[self.nInstance] = [self.curLat, self.curLon, self.curAlt]
            for p in lIP:
                if (get_distance_metres(p[0], p[1], self.curLat, self.curLon)< 2) and self.isOnMission:
                    print("Point detected by drone "+str(self.nInstance))
                    dist = get_distance_metres(self.curLat, self.curLon, self.takeOffLat, self.takeOffLon)
                    if self.logging : print("distance between GCS and IP : "+str(dist))
                    self.handleIP()
                    lIP.remove(p)
            time.sleep(0.5)


    def handleIP(self):
        ''' 
            Method called when a Interst Point is detected
        '''
        # First we stop the drone over the interest point      
        self.stopDrone()
        # After we check if the GCS is near enought to receive the video of the interest point
        if get_distance_metres(self.takeOffLat, self.takeOffLon, self.curLat, self.curLon)<100:
            if self.logging : print("The GCS is near enought to receive directly the video")
            self.wifiMsgGcs = "video from "+str(self.nInstance)
        else:
            # If the drone have an halow interface
            if self.isHalow:
                # send a message to GCS to inform it that an interest point is detected
                if self.logging : print("Inform GCS that there is an interest point") 
                # message structure : 'IP,self.lat; self.lon'
                self.halowMsg = "IP:"+str(self.curLat)+";"+str(self.curLon)
            else:
                # send broadcast wifi message to align all drones to join GCS
                # generate the list of points that have to be used
                neededPos = genPointsArray(self.curLat, self.curLon, self.curAlt, self.takeOffLat, self.takeOffLon, self.takeOffAlt)
                # Send broadcast message, yet you have to define it 
                self.sendBroadcastMsg()
                if self.logging: print("send broadcast wifi msg")


    def aat(self, aTargetAltitude):
        '''
            Arm and takeoff the drone

            Params :
                aTargetAltitude : targeted altitude
        '''
        # wainting for connection finished
        self.tDroneInstance.join()
            
        if self.logging : print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while self.running and not self.vehicle.is_armable :
            if self.logging : print(" Waiting for vehicle %i to initialise..."%self.nInstance)
            time.sleep(1)

        if self.logging : print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode    = VehicleMode("GUIDED")
        self.vehicle.armed   = True

        # Confirm vehicle armed before attempting to take off
        while self.running and not self.vehicle.armed:
            if self.logging : print(" Waiting for arming...")
            time.sleep(1)
            self.vehicle.mode    = VehicleMode("GUIDED")
            self.vehicle.armed   = True

        if self.logging : print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while self.running and not (self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95):
            if self.logging : print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            time.sleep(1)
        if self.logging : print("Reached target altitude")





    #################################### WIFI COM ####################################
    def sendWifiAll(self, msg):
        '''
            Method to send a broadcast wifi msg to all drone near the sender

            Params:
                msg : message to send
        '''
        global wifiCom, dronesPos
        dicMsg = {}
        if self.logging : print("send wifi all")
        for di in dronesPos:
            if di != self.nInstance:
                cT = dronesPos[di]
                if (distance(cT[0], cT[1], cT[2], self.curLat, self.curLon, self.curAlt) < 100):
                    dicMsg[di] = msg
        wifiCom[self.nInstance] = dicMsg
        if self.logging : print("drone "+str(self.nInstance) +" send : " + msg)


    def sendWifiOne(self, iD, msg):
        '''
            Method to send a wifi msg to a drone iD 

            Params:
                msg : message to send
        '''
        global wifiCom, dronesPos
        # If the specified ID is the one of this drone, means that it's a message for the GCS
        if iD == self.nInstance:
            self.wifiMsgGcs = msg
        # If it's not a message for the GCS
        else:     
            dicMsg = {}
            if self.logging : print("send wifi to "+str(iD))
            # get the position of the drone ID 
            posId = dronesPos[iD]
            # If the drone is under 100m away 
            if (get_distance_metres(posId[0], posId[1], self.curLat, self.curLon) < 100):
                dicMsg[iD] = msg
                wifiCom[self.nInstance] = dicMsg
                if self.logging : print("drone "+str(self.nInstance) +" send : " + msg)
            else:
                if self.logging : print("drone "+str(iD)+" is too far ("+str(dist)+" m)")


    def getWifiMsg(self):
        '''
             Method for the GCS to get wifi messages from drone
        '''
        self.tDroneInstance.join()
        self.tAat.join()
        dist = get_distance_metres(self.takeOffLat, self.takeOffLon, self.curLat, self.curLon)
        if dist<100:
            return self.wifiMsgGcs
        else:
            return None


    def handleReceiveWifi(self):
        '''
            method to search for new wifi messages
        '''
        global wifiCom
        self.tDroneInstance.join()
        oldGcsVal = None
        while self.running:
            # check for message from GCS
            if self.wifiMsgGcs != oldGcsVal:
                oldGcsVal = self.wifiMsgGcs
                if self.wifiMsgGcs == "VID:OK":
                    # The GCS inform the drone that it can start again its mission
                    self.startMission()
            # check for message from other drones
            for di in wifiCom:
                curDic = wifiCom[di]
                # for initialisation purpose :
                if di not in self.wifiMsgOld :
                    self.wifiMsgOld[di] = None
                if self.nInstance in curDic and self.wifiMsgOld[di] != curDic[self.nInstance]:
                    self.wifiMsgOld[di] = curDic[self.nInstance]
                    if self.logging : print(str(self.nInstance) + "get message : " + str(curDic[self.nInstance]) + " from : " + str(di))
                    sMsg = curDic[self.nInstance].split(":")
                    if "video from" in sMsg[0]:
                        if sMsg[1] == '': # if this drone is the last, try to send to the GCS
                            self.wifiMsgGcs = sMsg[0]
                        else:
                            cLink = sMsg[1].split(";")
                            first = cLink[0]
                            cLink.pop(0)
                            self.sendWifiOne(int(first),sMsg[0]+';'.join(cLink))
                    # if the drone doesn't have a wifi Halow interface
                    elif not self.isHalow:
                        theSender,theCommand, listOfNexts = self.defineBroadCastMsg()
                        # if broadcast message
                        if theSender != None:
                            pass # do things for commands on broadcast msg
                        
                        # the message may not be broacast
                        else:
                            pass # do things if the message is not for broascast purposes
            time.sleep(0.1)



    def defineBroadCastMsg(self, msg):
        sMsg = msg.split(":")
        sender = None
        command = ""
        listOfNexts = []
        if len(sMsg) == 3:
            sender = int(sMsg[0])
            command = str(sMsg[1])
            ssMsg = sMsg[2].split(";")
            for iDs in ssMsg:
                listOfNexts.append(int(iDs))
        return sender, command, listOfNexts



    def sendBroadcastMsg(self, msg):
        '''
            Method to send broadcast messages through Wifi to all drones that are less than 100 metres from the emmeters

            Params:
                    msg : messsage to send 
        '''
        global dronesPos, wifiCom
        availableDrones = []
        # check wich are the drones in a range of 100m from the sender, save their ID in availableDrones
        for droneId in dronesPos:
            if get_distance_metres(dronePos[droneId][0], dronePos[droneId][1], self.curLat, self.curLon)<100:
                availableDrones.append(droneId)
        # generate the string that we will send to those drones
        message = str(self.nInstance)+":"+str(msg)+":"
        listOfNotNotAvailables = ""
        for aD in dronesPos.keys():
            if aD not in availableDrones:
                listOfNotNotAvailables += str(aD)+";"
        msgToSend = message + listOfNotNotAvailables[::-1]
        if self.logging: print("broadcast msg send by "+str(self.nInstance)+" is : "+ msgToSend)
        for aD in availableDrones:
            self.sendWifiOne(aD, msgToSend)
            if self.logging : print("Send broadcast msg to "+str(aD)+" : "+msgToSend)
        if self.logging : print("End of broadcast sending for drone : "+str(self.nInstance))
            
            



    #################################### HALOW Com ####################################
    def handleReceiveHalow(self):
        '''
            Method searching for new halow messages
        '''
        self.tDroneInstance.join()
        self.tAat.join()
        while self.running:
            if self.isNewHalowMsg:
                self.isNewHalowMsg = False
                self.handleHalowMsg(self.halowMsg)
            time.sleep(0.1)


    def handleHalowMsg(self, msg):
        '''
            Method to react to Halow messages sended to this drone

            Params:
                msg : halow msg send to the drone 
        '''
        if self.logging : print("Halow message receive by drone "+str(self.nInstance)+" is "+str(msg))
        lMsg = msg.split(':')
        if self.logging : print("splited msg give : "+str(lMsg))
        if len(lMsg)==1:
            if msg == "STOP":
                # stop all drone movements
                self.stopDrone()
                if self.logging : print("drone "+str(self.nInstance)+" stopped")
            elif msg == "START":
                # start drone's movements
                self.startMission()
            elif msg == "POS":
                # answer the curent position of the drone
                self.halowMsg = "POS:"+str(self.curLat)+";"+str(self.curLon)+";"+str(self.curAlt)
            else:
                if self.logging : print("Message doesn't have the right format")
        elif len(lMsg)==2:
            if lMsg[0] == "ALIGN":
                # The drone is asked to be aligned by going to given point
                self.stopDrone()
                sMsg = lMsg[1].split(";")
                self.handleAlign(sMsg[0], sMsg[1])
            elif lMsg[0] == "ALIGNED":
                # List of the aligned drones to know to wich the video transmition have to be done
                sMsg = lMsg[1].split(";")
                first = sMsg[0]
                sMsg.pop(0)
                self.sendWifiOne(int(first),"video from "+str(self.nInstance)+":"+';'.join(sMsg))
            elif lMsg[0] == "VID":
                if lMsg[1] == "OK":
                    if self.logging : print("Video have been sent")
            else:
                if self.logging : print("Message doesn't have the right format")
        else:
            time.sleep(0.1)


    #################################### Drone mvmts ####################################

    def startMission(self):
        '''
            Method to start the mission
        '''
        # First we wait for the drone to initialise and takeoff
        self.tDroneInstance.join()
        self.tAat.join()
        # We check if a mission have been provided to the drone
        if len(self.wayPoints)<1:
            print("You have to import a mission first")
        # If the mission already started we just resume it
        elif self.tMission.is_alive():
            if not self.isOnMission:
                if self.logging : print("Drone "+str(self.nInstance)+" continue the mission")
                try:
                    self.sendCmdwControl(self.stopPos[0], self.stopPos[1], self.stopPos[2])
                except e:
                    if self.logging : print(e)
                self.isOnMission = True
                self.isStop = False
        # If the mission were not started, we start it
        else:
            if self.logging : print(str(self.nInstance) + " launch mission")
            self.isOnMission = True
            self.isStop = False
            self.tMission.start()

    
    def stopDrone(self):
        '''
            Method to stop the drone movements
        '''
        if self.logging : print("stop drone "+str(self.nInstance))
        # We stop the current mission
        self.isOnMission = False
        self.isStop = True # to stop the current command
        # Wait for the drone to exit it's check in sendCmdWCtrl()
        time.sleep(1)
        # Save the position where the drone stopped
        self.stopPos = [self.curLat, self.curLon, self.curAlt]
        self.isStop = False # to allow again to send new commands
        # Send the drone to it's position just to be shure it doesn't go to an anwanted position 
        self.vehicle.simple_goto(LocationGlobal(self.curLat, self.curLon, self.curAlt), airspeed=10)
        


    def sendCmdwControl(self, lat, lon, alt, speed = 30):
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
        if self.logging : print("drone "+ str(self.nInstance)+" goto : " + str(lat) + " " + str(lon) + " " + str(alt))
        self.vehicle.simple_goto(LocationGlobal(lat, lon, self.curAlt), airspeed = speed)
        while not self.isStop and (get_distance_metres(lat, lon, self.curLat, self.curLon)>1.2):
            dist = get_distance_metres(lat, lon, self.curLat, self.curLon)
            self.vehicle.simple_goto(LocationGlobal(lat, lon, self.curAlt), airspeed = speed)
            time.sleep(0.5)
    

    def handleWayPoints(self):
        '''
            Method to run the mission given to the drone
        '''
        # You should not use this method 
        i = 1
        maxId = len(self.wayPoints)
        if self.logging : print(" Nb commandes : " + str(maxId))
        while i<=maxId and self.running:
            if self.isOnMission:
                tempCmd = self.wayPoints[i]
                lat = tempCmd[0]
                lon = tempCmd[1]
                alt = tempCmd[2]
                self.sendCmdwControl(lat, lon, alt)
                if not self.isStop: #if we exit sendCmdwControl because isStop, we dont want to go to another command as we want to continue the actual task when the drone is not stop anymore
                    i += 1
        print("Mission finished by drone "+str(self.nInstance))
        self.vehicle.mode = VehicleMode("RTL")



    def handleAlign(self, lat, lon, alt="600"):
        '''
            Method to stop the current mission and send the drone to a point

            Params:
                lat : latitude of the destination point
                lon : longitude of the destination point
                alt : altitude of the destination point 
        '''
        # first we stop the drone
        self.stopDrone()
        # then we tell it where to go
        if self.logging : print("Drone : "+str(self.nInstance)+"  aligning")
        self.sendCmdwControl(float(lat), float(lon), float(alt))
        # Once it gets there, we inform the operator with a terminal print, and others drones with a wifi message
        if self.logging : print("Drone : "+str(self.nInstance)+"  aligned !")
        self.sendWifiAll("ok")
        # If the drone got an halow interface, we use it to inform the GCS and others drones
        if self.isHalow:
            self.halowMsg = "ALIGN:OK"
            
    


        
    #################################### Close the instance ####################################
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
        global winTerm
        os.system("xdotool windowclose %s"%winTerm)





###################################################################
if __name__=='__main__':
     #--home=-35.363261,149.165230,584,353
    ui1 = Vehicle(1)
    ui2 = Vehicle(2)

    ui1.start()
    ui2.start()

    time.sleep(10)
    ui1.sendWifiAll("coucou")
    time.sleep(10)
    ui2.printWifiCom()
    time.sleep(10)
    ui1.printDronesPos()
    ui2.printDronesPos()
    ui1.sendWifiAll("coucou")
    ui2.printWifiCom()


