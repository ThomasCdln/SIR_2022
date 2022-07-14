from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from detect_colision import CollisionHandler
from vehicle import Vehicle, getGcsPos
import random
import os
from Random_point_generator import RandomPointGenerator
import math 
import numpy as np
from threading import Thread


# CONSTANTS :
minLatIP = 0
maxLatIP = 0
minLonIP = 0
maxLonIP = 0
gcsLat = 0
gcsLon = 0
gcsAlt = 0
#############

# GLOBAL VARIABLES : 
vehicles = {}
lastSendHalow = {}
tHalow = None
continu = True
numberOfDetectedPoints = 0
alignDrones = {}
selectedDrones = []
####################

def handleWifiMsg():
    '''
        Method to handle classic wifi messages
        Run as a thread
    '''
    global vehicles, continu, numberOfDetectedPoints, selectedDrones
    while continu:
        for i in range(1, len(vehicles)+1):
            msg = vehicles[i].getWifiMsg()
            if msg != None:
                print("receive wifi msg from :"+str(i)+" msg is : "+str(msg))
                if "video" in msg:
                    print("GCS receive the video from the drone")
                    vehicles[i].wifiMsgGcs = "VID:OK"
                    numberOfDetectedPoints -= 1
                    if msg[-1] == str(i):
                        sendHalowMsg(i, "START")
                    else:
                        for d in selectedDrones:
                            sendHalowMsg(d, "START")
                    print("There are "+str(numberOfDetectedPoints)+" points left")
                else:
                    vehicles[i].wifiMsgGcs = None
        time.sleep(0.1)


def handleHalowMsg():
    '''
        Method to handle wifi halow messages
        Run as a thread
    '''
    global vehicles, continu, lastSendHalow
    rDrone = None
    rLat = None
    rLon = None
    fPart = None
    sPart = None
    while continu :
        for i in range(1, len(vehicles)+1):
            msg = vehicles[i].halowMsg
            if not (i in lastSendHalow): # for initialisation purpose only
                lastSendHalow[i] = None
            if msg != lastSendHalow[i] and msg!=None:
                print("receive wifi hallow msg from :"+str(i)+" msg is : "+str(msg))
                resetHalowMsg(i)
                lMsg = msg.split(':')
                if len(lMsg)==2:
                    if lMsg[0] == "IP":
                        # A drone have detected an IP
                        sMsg = lMsg[1].split(";")
                        print("IP detected by drone : "+str(i)+" at "+str(sMsg))
                        handleAlignR(sMsg[0], sMsg[1], i)
                    elif lMsg[0] == "ALIGN":
                        if lMsg[1] == "OK":
                        # The drone getted to asked position
                            print("drone "+str(i)+" is aligned")
                    elif lMsg[0] == "VID":
                        if lMsg[1] == "OK":
                            print("Video have been sent")
                    elif lMsg[0] == "POS":
                        sMsg = lMsg[1].split(";")
                        if len(sMsg)==3:
                            # The drone gives its position
                            print("Drone "+str(i)+" position is : "+str(sMsg))
                        else:
                            print("Drone "+str(i)+" doesn't have the right position message")
                    else:
                        print("Message doesn't have the right format")
                else:
                    time.sleep(0.1)
                
                    


                    
def handleAlignR(aLat, aLon, rId):
    '''
        Method to handle the alignment for creating the multi hop network
    '''
    global gcsLat, gcsLon, vehicles, gcsAlt, alignDrones, selectedDrones
    time.sleep(1)
    gcsLat, gcsLon, gcsAlt = getGcsPos()
    print("gcsLat, gcsLon, gcsAlt : "+str(gcsLat) + " " + str(gcsLon) + " " + str(gcsAlt))
    selectedDrones = []
    selectedDrones.append(rId)                     
    dronesPositions = getDronesPosHalow()
    points = genPointsArray(gcsLat, gcsLon, gcsAlt, float(aLat), float(aLon), gcsAlt)
    print(points)
    print(dronesPositions)
    for p in points:
        point = points[p]
        rLat = point[0]
        rLon = point[1]
        bestDist = 1000
        bestId = 0
        for iD in dronesPositions:
            if iD not in selectedDrones:
                dPos = dronesPositions[iD]
                dist = distance(rLat, rLon, dPos[0], dPos[1])
                if bestDist > dist:
                    bestDist = dist
                    bestId = iD
        selectedDrones.append(bestId)
        #######################################################
        alignDrones[bestId] = [rLat,rLon]
        #######################################################
        
    # sending order to selected drones to align
    print("Align drones : " + str(alignDrones))
    for aD in alignDrones:
        rPos = alignDrones[aD]
        sendHalowMsg(aD, "ALIGN:"+str(rPos[0])+";"+str(rPos[1])) # ask drone to get to position
    #####################################################################################
    # check if selected drones are at position
    arrived = False
    for aD in alignDrones:
        while not arrived:
            if vehicles[aD].halowMsg == "ALIGN:OK":
                arrived = True
                break
            time.sleep(0.01)
        arrived = False
    print("All drones gets alligned")
    # sending the list of aligned drones to the requester drone
    msg = "ALIGNED:"
    for iD in alignDrones:
        msg += str(iD)+";"
    sendHalowMsg(rId, msg[:-1]) # we send the list of aligned drones to the requester ( and drop the last ;)
        
                    

    
    
def getDronesPosHalow(iD=None):
    '''
        Method that asks all drones their position

        Params:
                iD : iD of a drone that we want to skip
                return : a dictionnary containning position of all drones excepted iD if specified
    '''
    global vehicles
    pos = {}
    sendAllExceptOneHalow("POS", None) # ask drones for their position
    # getting awnsers
    for i in range(1, len(vehicles)+1):
        if i!=iD:
            msg = vehicles[i].halowMsg
            while msg==None or msg=="POS":
                time.sleep(0.1)
                msg = vehicles[i].halowMsg
            print("msg from "+str(i)+" is : "+str(msg))
            tmp = msg.split(":")
            if tmp[0] == "POS":
                sPart = tmp[1]
                tmp = sPart.split(";")
                pos[i] = float(tmp[0]), float(tmp[1])
    return pos
                    
                    

def sendAllExceptOneHalow(msg, excepted):
    '''
        Method to send halow messages to all drones excepted one

        Params:
                msg : the message to send
                excepted : id of the drone excepted
    '''
    global vehicles
    print("sending : "+msg+" to all drones excepted : "+str(excepted))
    for i in range(1, len(vehicles)+1):
        if i != excepted:
            sendHalowMsg(i, msg)

                
def sendHalowMsg(iD, msg):
    '''
        Method to send halow message to one drone

        Params:
                iD : ID of the drone that we want to send the message
                msg : message to send to the drone 
    '''
    global vehicles, lastSendHalow
    lastSendHalow[iD] = msg
    vehicles[iD].halowMsg = msg
    vehicles[iD].isNewHalowMsg = True


def resetHalowMsg(iD):
    '''
        Method to reset halow messages of a drone

        Params:
                iD : id of the drone for wich we want to reset the messages
    '''
    global vehicles, lastSendHalow
    vehicles[iD].halowMsg = None
    vehicles[iD].isNewHalowMsg = False



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

def distance(lat1, lon1, lat2, lon2):
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = lat2 - lat1
    dlong = lon2 - lon1
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


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
            print("else")
            ui[i] = x,y
            i+=1
            newX = x
            newY = y
    print("took : "+str(time.time()-begin))
    return ui
    

def writePOIFile(poi):
    '''
        Method for writing a new connection address wich be use by Mission Planner

        Params:
            newAdd : Address to add to connectionList.txt
    '''
    f = open("/mnt/hgfs/Connect/POI.txt", "a") # Path to the shared directory, depend on the configuration of your VM
    for i in range(0, len(poi)):
        line = str(poi[i][0])+"\t"+str(poi[i][1])+"\t"+str(i)+"\n"
        print(line)
        f.write(line)
    f.close()

#--home=.362149
lat1 = -35.363261
lon1 = 149.165230
alt1 = 584
lat2 = -35.363280
lon2 = 149.165220
alt2 = 584



def main():
    global numberOfDetectedPoints, vehicles, continu, gcsLat, gcsLon, gcsAlt

    # Barcelona airport : 41.3061778,2.1050781,0.162556,353  -- didn't found how to get the fourth parameter for now
    home = [41.3061778,2.1050781,0.162556,353]
    # if a connection file exist, we delete it -> when drones instances are created, they create the file and implement it
    if os.path.exists("/mnt/hgfs/Connect/connectionList.txt"): # Path to the shared directory, depend on the configuration of your VM
        os.remove("/mnt/hgfs/Connect/connectionList.txt") # Path to the shared directory, depend on the configuration of your VM
    if os.path.exists("/mnt/hgfs/Connect/POI.txt"): # Path to the shared directory, depend on the configuration of your VM
        os.remove("/mnt/hgfs/Connect/POI.txt") # Path to the shared directory, depend on the configuration of your VM

    # create new set of interest points
    #rpg = RandomPointGenerator(-35.3634182, -35.3621443, 149.1651605, 149.1651776) # Have to set (min & max) latitude and longitude
    #lIP = rpg.getMultiplePoints(10) # Get random points of interest
    lIP = [[-35.36215420, 149.16508380]]
    numberOfDetectedPoints = len(lIP)

    # Write the interest points in a file
    writePOIFile(lIP)
    

    # Create drones instances
    for i in range(1, 4):
        #vehicles[i] = Vehicle(i, lIP)
        # vehicles[i] = Vehicle(i, lIP, home)
        # vehicles[i].isHalow = True  # the drone have Wifi halow
        #vehicles[i] = Vehicle(i, lIP, True) # for debug purposes
        vehicles[i] = Vehicle({"nInstance":i, "lIP":lIP, "logging":True})
        vehicles[i].isHalow = True # setted to true if the drone have an Halow interface, False otherwise
        print("instance " + str(i) + " created")

    # Launch GCS halow messages handler 
    tHalow = Thread(target=handleHalowMsg, args=())
    tHalow.start()
    # Launch GCS wifi messages handler
    tWifi = Thread(target=handleWifiMsg, args=())
    tWifi.start()
    
    # Initiate the collision handler
    ch = CollisionHandler(vehicles)
    ch.start()

    


    # set and start missions
    for v in vehicles:
        vehicles[v].setWayPointsFromFile("/mnt/hgfs/Connect/v"+str(v)+".waypoints") # Path to the shared directory, depend on the configuration of your VM
        vehicles[v].startMission()


        
    # TEST
    
    # close
    #for v in vehicles:
    #   vehicles[v].close()
    #vehicles[0].closeWin()
    #exit()



if __name__=='__main__':
    main()

# usefull inf : path to share folder : /mnt/hgfs/Connect/



