from math import pi,sqrt,atan2,radians,degrees,cos,sin,asin,ceil
import random

def getBearing(lat1, lng1, lat2, lng2):
	diffLng = lng2 - lng1
	diffLat = lat2 - lat1
	bearing = degrees(90+atan2(-diffLat, diffLng))
  # +90 because trigo "north" is geographic east    
	return (bearing+360)%360
	#to go from the [-180;180] range to the [0;360] range
	
def getDistance(lat1, lng1, lat2, lng2, alt=6731000 ) :
	if alt<6731000:
		alt+=6731000
	#écart en degré
	diffLat = abs(lat1-lat2)
	diffLng = abs(lng1-lng2)
	#écart en mètre
	distLat = alt*(pi/180)*diffLat
	distLng = alt*(pi/180)*diffLng
	#distance en les deux points
	ecart=sqrt(distLat**2+distLng**2)
	return ecart

def nbDroneMin(lat1, lng1, lat2, lng2, alt=6731000 ) :
	#si le GCS est dans le coin de la piste :
	return max(1,ceil(getDistance(lat1, lng1, lat2, lng2, alt)/100)-1)
	#si il est au centre: return ceil(max(1,distanceSol(lat1, lng1, lat2, lng2, alt )/100)/2)

def newProjectedPoint (lat, lng, bearing, distance, alt=6731000):
	if alt<6731000:
		alt+=6731000
	pos=[]
	#convertion en rad
	lat = radians(lat)
	lng = radians(lng)
	bearing = radians(bearing)
	#calcul des coordonees
	pos.append(degrees(asin(sin(lat)*cos(distance/alt) + cos(lat)*sin(distance/alt)*cos(bearing))))
	pos.append(degrees(lng+atan2(sin(bearing)*sin(distance/alt)*cos(lat), cos(distance/alt)-sin(lat)*sin(pos[0]))))
	#reconvertion en degres
	return pos

def newProjectedPoI (lat1, lng1, lat2, lng2, lat3, lng3, alt=673100):
	if alt<6731000:
		alt+=6731000
	#définir l'axe de la piste
	rwyForward = getBearing(lat1,lng1,lat2,lng2)
	rwyRight = getBearing(lat1,lng1,lat3,lng3)
	#définir les dimentions de la piste (repère cardinal relatif)
	distForward = getDistance(lat1,lng1,lat2,lng1,alt)
	distRight = getDistance(lat1, lng1, lat3, lng3, alt)
	#calcul des coordonees
	pos=[]
	pos.append(newProjectedPoint(lat1,lng1,rwyForward,random.uniform(0,distForward))[0])
	pos.append(newProjectedPoint(lat1,lng1,rwyRight,random.uniform(0,distRight))[1])
	return pos

def createChain(lat1, lng1, lat2, lng2, alt=673100):
	if alt<6731000:
		alt+=6731000
	pointList={}
	bearing = getBearing(lat1, lng1, lat2, lng2)
	totalDist = getDistance(lat1, lng1, lat2, lng2, alt)
	neededNumber=totalDist//100+1
	step=totalDist/neededNumber
	currentDist=0
	currentLat= lat1
	currentLng= lng1
	numIter=0
	while currentDist < totalDist:
		point = newProjectedPoint(currentLat, currentLng, bearing, step,  alt)
		currentLat=point[0]
		currentLng=point[1]
		currentDist+=step
		pointList[numIter]=point
		numIter+=1
	return pointList

def pathMaker (lat1, lng1, lat2, lng2, lat3, lng3, alt, scanWidth=2, altVol=5, nbDrone=1):
	if alt<6731000:
		alt+=6731000
	pointList = {}
	numEtape = 0
	#définir l'axe de la piste
	rwyForward = getBearing(lat1,lng1,lat2,lng2)
	rwyBackward = getBearing(lat2,lng2,lat1,lng1)
	rwyRight = getBearing(lat1,lng1,lat3,lng3)
	#définir les dimentions de la piste (repère cardinal relatif)
	distForward = getDistance(lat1,lng1,lat2,lng2,alt)
	distRight = getDistance(lat1,lng1,lat3,lng3,alt)
	#définir la pose de départ
	altVol += alt
	posDepart = newProjectedPoint(lat1, lng1, altVol, (rwyForward+45+360)%360, scanWidth/2*sqrt(2))
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
			point = newProjectedPoint(latDrone, lngDrone, rwyForward, distForward-scanWidth)
			dir=3
			prevDir=1
			latDrone=point[0]
			lngDrone=point[1]
			point.append(altVol)
			pointList[numEtape]=point
			numEtape+=1
		elif dir==2: #vers le sud
			point = newProjectedPoint(latDrone, lngDrone, rwyBackward, distForward-scanWidth)
			dir=3
			prevDir=2
			latDrone=point[0]
			lngDrone=point[1]
			point.append(altVol)
			pointList[numEtape]=point
			numEtape+=1
		elif prevDir==1: #vers l'est depuis traj sud>nord
			point = newProjectedPoint(latDrone, lngDrone, rwyRight, scanWidth)
			dir=2
			latDrone=point[0]
			lngDrone=point[1]
			point.append(altVol)
			pointList[numEtape]=point
			numEtape+=1
			currdistRight+=scanWidth
		else : #vers l'est depuis traj nord>sud
			point = newProjectedPoint(latDrone, lngDrone, rwyRight , scanWidth)
			dir=1
			latDrone=point[0]
			lngDrone=point[1]
			point.append(altVol)
			pointList[numEtape]=point
			numEtape+=1
			currdistRight+=scanWidth
	return pointList


def poiGenerator(lat1, lng1, lat2, lng2, lat3, lng3, nbPoints,alt=6731000):
	if alt<6731000:
		alt+=6731000
	poiList=[]
	for i in range(nbPoints):
		poiList.append(newProjectedPoI(lat1, lng1, lat2, lng2, lat3, lng3, alt))
	return poiList

print(poiGenerator(41.2823216,2.0735461,41.2818459,2.0738196,41.2926877,2.1037970, 10,5))
