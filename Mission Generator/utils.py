from math import cos, sin, tan, atan2, sqrt, radians, degrees, pi, log, fmod,ceil
import random

radius_of_earth = 6378100.0 # in meters

def gps_distance(lat1, lon1, lat2, lon2):
    '''distance between two points in meters'''
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    lon1 = radians(lon1)
    lon2 = radians(lon2)

    if abs(lat2-lat1) < 1.0e-15:
        q = cos(lat1)
    else:
        q = (lat2-lat1)/log(tan(lat2/2+pi/4)/tan(lat1/2+pi/4))
    d = sqrt((lat2-lat1)**2 + q**2 * (lon2-lon1)**2)
    return d * radius_of_earth

def gps_bearing(lat1, lon1, lat2, lon2):
    '''return rhumb bearing between two points in degrees (range 0-360)'''
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    tc = -fmod(atan2(lon1-lon2,log(tan(lat2/2+pi/4)/tan(lat1/2+pi/4))),2*pi)
    if tc < 0:
        tc += 2*pi
    return degrees(tc)

def constrain(v, minv, maxv):
    if v < minv:
        v = minv
    if v > maxv:
        v = maxv
    return v

def gps_newpos(lat, lon, bearing, distance):
    '''extrapolate latitude/longitude given a heading and distance
    along rhumb line thanks to http://www.movable-type.co.uk/scripts/latlong.html
    '''
    lat1 = constrain(radians(lat), -pi/2+1.0e-15, pi/2-1.0e-15)
    lon1 = radians(lon)
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
    dlon = -d*sin(tc)/q
    lon = fmod(lon1+dlon+pi,2*pi)-pi
    return [degrees(lat), degrees(lon)]

def nbDroneMin(lat1, lng1, lat2, lng2, alt=6378100 ) :
	#si le GCS est dans le coin de la piste :
	return max(1,ceil(gps_distance(lat1, lng1, lat2, lng2)/100)-1)
	#si il est au centre: return ceil(max(1,distanceSol(lat1, lng1, lat2, lng2, alt )/100)/2)

def pathMaker (lat1, lng1, lat2, lng2, lat3, lng3, alt, scanWidth=2, altDrone=5, nbDrone=1):
	pointList = {}
	numEtape = 0
	#définir l'axe de la piste
	rwyForward = gps_bearing(lat1,lng1,lat2,lng2)
	rwyBackward = gps_bearing(lat2,lng2,lat1,lng1)
	rwyRight = gps_bearing(lat1,lng1,lat3,lng3)
	#définir les dimentions de la piste (repère cardinal relatif)
	distForward = gps_distance(lat1,lng1,lat2,lng2)
	distRight = gps_distance(lat1,lng1,lat3,lng3)
	#définir la pose de départ
	altDrone += alt
	posDepart = gps_newpos(lat1, lng1, (rwyForward+45+360)%360, scanWidth/2*sqrt(2))
	pointList[numEtape] = posDepart
	numEtape+=1
	#actualiser la position du drone
	latDrone=posDepart[0]
	lngDrone=posDepart[1]
	#definir les points du trajet
	dir=1 #current bearing : 1 rwyForward, 2 -rwyForward, 3 perpendiculaire
	currdistRight=0
	while currdistRight<distRight-scanWidth:
		if dir==1: #vers le nord
			point = gps_newpos(latDrone, lngDrone, rwyForward, distForward-scanWidth)
			dir=3
			prevDir=1
			latDrone=point[0]
			lngDrone=point[1]
			point.append(altDrone)
			pointList[numEtape]=point
			numEtape+=1
		elif dir==2: #vers le sud
			point = gps_newpos(latDrone, lngDrone, rwyBackward, distForward-scanWidth)
			dir=3
			prevDir=2
			latDrone=point[0]
			lngDrone=point[1]
			point.append(altDrone)
			pointList[numEtape]=point
			numEtape+=1
		elif prevDir==1: #vers l'est depuis traj sud>nord
			point = gps_newpos(latDrone, lngDrone, rwyRight, scanWidth)
			dir=2
			latDrone=point[0]
			lngDrone=point[1]
			point.append(altDrone)
			pointList[numEtape]=point
			numEtape+=1
			currdistRight+=scanWidth
		else : #vers l'est depuis traj nord>sud
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
	rwyForward = gps_bearing(lat1,lng1,lat2,lng2)
	rwyRight = gps_bearing(lat1,lng1,lat3,lng3)
	missionList={}
	nbDrone=2*nbDroneMin(lat2,lng2,lat3,lng3,alt)
	print (str(nbDrone)+" zones")
	widthZone= gps_distance(lat1,lng1,lat3,lng3)
	longTot=gps_distance(lat1,lng1,lat2,lng2)
	longZone=longTot/nbDrone
	oldLPoint=[lat1,lng1]
	oldRPoint=[lat3,lng3]
	newPoint=[]
	newPoint=gps_newpos(lat1, lng1, rwyForward,longZone)
	for i in range(nbDrone):
		print("left"+str(oldLPoint))
		print("right"+str(oldRPoint))
		print("new"+str(newPoint)+"\n")
		missionList[i]=pathMaker(oldLPoint[0],oldLPoint[1],newPoint[0],newPoint[1],oldRPoint[0],oldRPoint[1],alt)
		oldLPoint=newPoint
		oldRPoint=gps_newpos(oldLPoint[0], oldLPoint[1], rwyRight, widthZone)
		newPoint=gps_newpos (oldLPoint[0], oldLPoint[1], rwyForward, longZone)
	return missionList
	
def newProjectedPoI (lat1, lng1, lat2, lng2, lat3, lng3, alt=6378100):
	#définir l'axe de la piste
	rwyForward = gps_bearing(lat1,lng1,lat2,lng2)
	rwyRight = gps_bearing(lat1,lng1,lat3,lng3)
	#définir les dimentions de la piste (repère cardinal relatif)
	distForward = gps_distance(lat1,lng1,lat2,lng2)
	distRight = gps_distance(lat1, lng1, lat3, lng3)
	#calcul des coordonees
	pos=[]
	randDistFwd=random.uniform(0,distForward)
	randDistRight=random.uniform(0,distRight)
	pos.append(gps_newpos(lat1,lng1,rwyForward,randDistFwd)[0])
	pos.append(gps_newpos(lat1,lng1,rwyRight,randDistRight)[1])
	return pos
	
def poiGenerator(lat1, lng1, lat2, lng2, lat3, lng3, nbPoints,alt=6378100):
	poiList=[]
	for i in range(nbPoints):
		poiList.append(newProjectedPoI(lat1, lng1, lat2, lng2, lat3, lng3, alt))
	return poiList

poiNumber=3

mission2 =zones (41.2930275,2.0655853, 41.2934568,2.0668057, 41.2924270,2.0659500)
poi2 = poiGenerator(41.2930275,2.0655853, 41.2934568,2.0668057, 41.2924270,2.0659500,poiNumber)

mission4 = zones (41.2930275,2.0655853, 41.29388,2.0680529, 41.2924270,2.0659500)
poi4 = poiGenerator (41.2930275,2.0655853, 41.29388,2.0680529, 41.2924270,2.0659500,poiNumber)

mission8= zones (41.2930275,2.0655853, 41.2947546,2.0703864, 41.2924270,2.0659500)
poi8 = poiGenerator (41.2930275,2.0655853, 41.2947546,2.0703864, 41.2924270,2.0659500,poiNumber)
