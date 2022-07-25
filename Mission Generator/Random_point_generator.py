import random
 

class RandomPointGenerator():
    def __init__(self, *args):
        self.isBoundaries = False
        if len(args) == 4:
            self.minLat = args[0]
            self.maxLat = args[1]
            self.minLon = args[2]
            self.maxLon = args[3]
            self.isBoundaries = True

    def setPointBoundaries(self, minLat, maxLat, minLon, maxLon):
        self.minLat = minLat
        self.maxLat = maxLat
        self.minLon = minLon
        self.maxLon = maxLon
        self.isBoundaries = True

    def newPos(self):
        if self.isBoundaries :
            randomLat = random.uniform(self.minLat, self.maxLat)
            randomLon = random.uniform(self.minLon, self.maxLon)
            print("Point generated at %f %f", randomLat, randomLon)
            return [randomLat, randomLon]
        else :
            print("Cannot create new position : No boundaries defined")

    def getMultiplePoints(self, nPoints):
        pts = []
        for n in range(0, nPoints):
            pts.append(self.newPos())
        return pts

if __name__=='__main__':
    ui = RandomPointGenerator(1, 4, 5, 8)
    slt = ui.getMultiplePoints(5)
    print(slt)
    
            


        
          
