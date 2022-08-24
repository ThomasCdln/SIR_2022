from math import cos, sin, tan, atan2, sqrt, radians, degrees, pi, log, fmod,ceil
import random

radius_of_earth = 6378100.0 # in meters

def gps_distance(lat1, lng1, lat2, lng2):
    '''distance between two points in meters'''
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    lng1 = radians(lng1)
    lng2 = radians(lng2)

    if abs(lat2-lat1) < 1.0e-15:
        q = cos(lat1)
    else:
        q = (lat2-lat1)/log(tan(lat2/2+pi/4)/tan(lat1/2+pi/4))
    d = sqrt((lat2-lat1)**2 + q**2 * (lng2-lng1)**2)
    return d * radius_of_earth

def gps_bearing(lat1, lng1, lat2, lng2):
    '''return rhumb bearing between two points in degrees (range 0-360)'''
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    lng1 = radians(lng1)
    lng2 = radians(lng2)
    tc = -fmod(atan2(lng1-lng2,log(tan(lat2/2+pi/4)/tan(lat1/2+pi/4))),2*pi)
    if tc < 0:
        tc += 2*pi
    return degrees(tc)

def constrain(v, minv, maxv):
    if v < minv:
        v = minv
    if v > maxv:
        v = maxv
    return v

def gps_newpos(lat, lng, bearing, distance):
    '''extrapolate latitude/lenghtitude given a heading and distance along rhumb line
    '''
    lat1 = constrain(radians(lat), -pi/2+1.0e-15, pi/2-1.0e-15)
    lng1 = radians(lng)
    tc = radians(-bearing)
    d = distance/radius_of_earth

    lat = lat1 + d * cos(tc)
    lat = constrain(lat, -pi/2 + 1.0e-15, pi/2 - 1.0e-15)
    if abs(lat-lat1) < 1.0e-15:
        q = cos(lat1)
    else:
        try:
            dphi = log(tan(lat/2+pi/4)/tan(lat1/2+pi/4))
        except Exception:
            print(degrees(lat),degrees(lat1))
            raise
        q = (lat-lat1)/dphi
    dlng = -d*sin(tc)/q
    lng = fmod(lng1+dlng+pi,2*pi)-pi
    return [degrees(lat), degrees(lng)]

def nbDroneMin(lat1, lng1, lat2, lng2, alt=6378100 ) :
	#return the minimum number of drone when the GCS in the corner of the runway :
	return max(1,ceil(2,5*gps_distance(lat1, lng1, lat2, lng2)/100)-1)
	
def pathMaker (lat1, lng1, lat2, lng2, lat3, lng3, alt, scanWidth=2, altDrone=5, nbDrone=1):
	'''
	return a dictionnary containing the mission from a drone
	the corner have to be in inputed in the following way :
	
	corner 1-----------------------------------------corner 2
	|							|
	|		--> forward -->				|
	|							|
	corner 3------------------------------------------------|
	
	'''
	pointList = {}
	numEtape = 0
	#define runway axis
	rwyForward = gps_bearing(lat1,lng1,lat2,lng2)
	rwyBackward = gps_bearing(lat2,lng2,lat1,lng1)
	rwyRight = gps_bearing(lat1,lng1,lat3,lng3)
	#define runaway dimensions
	distForward = gps_distance(lat1,lng1,lat2,lng2)
	distRight = gps_distance(lat1,lng1,lat3,lng3)
	#define the starting position
	altDrone += alt
	posDepart = gps_newpos(lat1, lng1, (rwyForward+45+360)%360, scanWidth/2*sqrt(2))
	pointList[numEtape] = posDepart
	numEtape+=1
	#actualsie memorised position
	latDrone=posDepart[0]
	lngDrone=posDepart[1]
	#define path points
	dir=1 #current bearing : 1 rwyForward, 2 -rwyForward, 3 perpendiculaire
	currdistRight=0
	while currdistRight<distRight-scanWidth:
		if dir==1: #forward
			point = gps_newpos(latDrone, lngDrone, rwyForward, distForward-scanWidth)
			dir=3
			prevDir=1
			latDrone=point[0]
			lngDrone=point[1]
			point.append(altDrone)
			pointList[numEtape]=point
			numEtape+=1
		elif dir==2: #backward
			point = gps_newpos(latDrone, lngDrone, rwyBackward, distForward-scanWidth)
			dir=3
			prevDir=2
			latDrone=point[0]
			lngDrone=point[1]
			point.append(altDrone)
			pointList[numEtape]=point
			numEtape+=1
		elif prevDir==1: #sideways from forwrd
			point = gps_newpos(latDrone, lngDrone, rwyRight, scanWidth)
			dir=2
			latDrone=point[0]
			lngDrone=point[1]
			point.append(altDrone)
			pointList[numEtape]=point
			numEtape+=1
			currdistRight+=scanWidth
		else : #sideways from backward
			point = gps_newpos(latDrone, lngDrone, rwyRight , scanWidth)
			dir=1
			latDrone=point[0]
			lngDrone=point[1]
			point.append(altDrone)
			pointList[numEtape]=point
			numEtape+=1
			currdistRight+=scanWidth
	return pointList

def zones(lat1, lng1, lat2, lng2, lat3, lng3,alt=6378100):
		'''
	return a dictionnary of dictionnaries containing the missions from all the drones
	the corner have to be in inputed in the following way :
	
	corner 1-----------------------------------------corner 2
	|							|
	|		--> forward -->				|
	|							|
	corner 3------------------------------------------------|
	
	'''

	rwyForward = gps_bearing(lat1,lng1,lat2,lng2)
	rwyRight = gps_bearing(lat1,lng1,lat3,lng3)
	missionList={}
	nbDrone=fnbDroneMin(lat2,lng2,lat3,lng3,alt)
	print (str(nbDrone)+" zones")
	widthZone= gps_distance(lat1,lng1,lat3,lng3)
	lenghtTot=gps_distance(lat1,lng1,lat2,lng2)
	lenghtZone=lenghtTot/nbDrone
	oldLPoint=[lat1,lng1]
	oldRPoint=[lat3,lng3]
	newPoint=[]
	newPoint=gps_newpos(lat1, lng1, rwyForward,lenghtZone)
	for i in range(nbDrone):
		print("left"+str(oldLPoint))
		print("right"+str(oldRPoint))
		print("new"+str(newPoint)+"\n")
		missionList[i]=pathMaker(oldLPoint[0],oldLPoint[1],newPoint[0],newPoint[1],oldRPoint[0],oldRPoint[1],alt)
		oldLPoint=newPoint
		oldRPoint=gps_newpos(oldLPoint[0], oldLPoint[1], rwyRight, widthZone)
		newPoint=gps_newpos (oldLPoint[0], oldLPoint[1], rwyForward, lenghtZone)
	return missionList
	
def newProjectedPoI (lat1, lng1, lat2, lng2, lat3, lng3):
	#define runway axis
	rwyForward = gps_bearing(lat1,lng1,lat2,lng2)
	rwyRight = gps_bearing(lat1,lng1,lat3,lng3)
	#define runway dimensions
	distForward = gps_distance(lat1,lng1,lat2,lng2)
	distRight = gps_distance(lat1, lng1, lat3, lng3)
	#generation of a random position
	pos=[]
	randDistFwd=random.uniform(0,distForward)
	randDistRight=random.uniform(0,distRight)
	#creation of the outputted array
	pos.append(gps_newpos(lat1,lng1,rwyForward,randDistFwd)[0])
	pos.append(gps_newpos(lat1,lng1,rwyRight,randDistRight)[1])
	return pos
	
def poiGenerator(lat1, lng1, lat2, lng2, lat3, lng3, nbPoints):
	'''
	return an array of point of interest
	the corner have to be in inputed in the following way :
	
	corner 1-----------------------------------------corner 2
	|							|
	|		--> forward -->				|
	|							|
	corner 3------------------------------------------------|
	
	'''
	poiList=[]
	for i in range(nbPoints):
		poiList.append(newProjectedPoI(lat1, lng1, lat2, lng2, lat3, lng3))
	return poiList
F
