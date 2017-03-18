import numpy as np
# A* search
def bStarExpend(frontier = None, listPosition = []):
    if frontier == None:
        frontier = {}
        # get all points distances from start
        minValueForPoint = 10000
        for (index, point) in enumerate(self.goodPoints):
            if (index not in self.visitedGoodPoints):
                travelTime = self.travelTime(point)
                valueForPoint = self.valueFunction(point, None, travelTime)

                frontier[index] = {leafPosition: index, value: valueForPoint, path:[index], travelTime: travelTime, nextDistances:{}}
                if (valueForPoint < minValueForPoint):
                    bestNext = index
        list.append(bestNext)
    else:
        minTravelTime = 100000
        minValueForPoint = 100000
        for (frontierIndex,frontierData) in frontier.items(): # TODO cache the results
            for (index, value) in enumerate(self.goodPoints):
                if ((index not in listPosition) and (index not in self.visitedGoodPoints)):
                    if (index in hasattr(frontierData['nextDistances'], index)):
                        travelTime = fromPoint.travelTime + self.travelTime(point, self.goodPoints[frontierData.leafPosition])
                        valueForPoint = fromPoint.value + self.valueFunction(point, fromPoint, travelTime)
                        frontier[frontierIndex]['nextDistances'][index] = {travelTime: travelTime, valueForPoint: valueForPoint}
                    else:
                        travelTime = frontierData['nextDistances'][index]['travelTime']
                        valueForPoint = frontierData['nextDistances'][index]['valueForPoint']

                    #frontier.append()
                    if (valueForPoint < minValueForPoint):
                        bestNext = index
                        bestFrontiere = frontierIndex
                        bestValue = valueForPoint
                        bestTravelTime = travelTime

                    if (travelTime < minTravelTime):
                        minTravelTime = travelTime

        list.append(bestNext)
        newPath = frontier[bestFrontiere]['path'].append(bestNext)

        frontier[bestFrontiere] = {leafPosition: bestNext,
        value: valueForPoint,
        path: newPath,
        travelTime: frontier[bestFrontiere]['travelTime'] + bestTravelTime,
        nextDistances: {}}

    if (travelTime > self.maxDuration):
        return list
    else:
        return bStarExpend(frontier, listPosition)

# self.maxDuration

def getNonVisitedGoodPointsIndex(self):
    # loop through positive points TODO replace by array of non visited
    points = []
    for (index, point) in enumerate(self.goodPoints):
        if index in self.visitedGoodPoints:
            points.append(index)
        pass

    return points



# Value function
def valueFunction(forPoint, fromPoint, travelTime = None):
    if (travelTime is None):
        travelTime = self.travelTime(point, fromPoint)

    V = travelTime + self.remainainingTimeForPoint(forPoint)

    return V

def travelTime(forPoint, fromPoint):
    V = self.timeToTurnTo(forPoint, fromPoint) + self.timeToGoTo(forPoint, fromPoint)

    return V

# Model
def timeToGoTo(self, target, fromPoint = None):
    if fromPoint is None:
        fromPoint = self.currentPositionPoint()

    return self.euclidianDistance(target - fromPoint)/self.speed

def timeToTurnTo(self, point, fromPoint = None):
    if fromPoint is None:
        fromPoint = self.currentPositionPoint()

    angle = self.getTargetAngle(point, fromPoint)

    return (angle / angularSpeed)

def approximateTimeToExploreAera(self,center, distance = None):
    # based on a model of identically distributed targets in the aeras
    # and the robot roing one of the edge then 2 90 degree turns.
    if (distance is None):
        return np.sqrt((math.pi*math.pow(self.remainingDistance,2)/self.pointCloserThan(self.remainingDistance(), center)))/self.speed
    else:
        return np.sqrt((math.pi*math.pow(distance,2)/self.pointCloserThan(distance, center)))/self.speed

def remainainingTimeForPoint(self, center = None):
    if center is None:
        center = self.currentPositionPoint()

    remainingPoints = 45 - len(self.visitedGoodPoints) # TODO replace by number of points
    vicinityPoint = self.pointCloserThan(self.remainingDistance(), center)

    return self.averageDistanceForDensity(self.remainingDensityForPoint(center))*(remainingPoints - vicinityPoint)


# Density
def averageDistanceForDensity(density):
    # assuming flat distribution over the surface
    return np.sqrt(1/density)

def getDensityGrid(self, bins = 10):
    grid = np.zeros((10, 10))
    cellWidth = self.max_x / bins
    cellHeight = self.max_y / bins

    for x in range(0,bins):
        for y in range (0,bins):
            grid[x,y] = getNumberOfPointInCell()

    return grid

def pointsInCell (self, cell, onlyNonVisited = True):
    points = []
    for point in self.getNonVisitedGoodPoints():
        if (self.isPointInCell(point, cell)):
            points.append(point)

    return points

def isPointInCell(self, point, cell):
    px = point[0]
    py = point[1]
    cellx = cell[0]
    celly =  cell[1]
    cellWidth = cell[2]
    cellHeight = cell[3]

    if ((py > celly) and (py <= celly + cellHeight) and (px> cellx) and (px <= cellx+cellWidth)):
        return True

def getDensityForPoint(self, center = None):
    # either you place the point in the grid a get the value of the grid
    # or you compute the local density
    # density radius should depend on remaining time
    remainingPoints = 45 - len(self.visitedGoodPoints) # TODO replace by number of points
    vicinityPoint = self.pointCloserThan(self.remainingDistance(), center)

    return (len(vicinityPoint)/remainingPoints)

def remainingDensityForPoint(self, center = None):
    return 1 - getDensityForPoint(center)


def pointCloserThan(self, distance, fromPoint = None):
    if fromPoint is None:
        fromPoint = self.currentPositionPoint()

    points = []
    for point in self.getNonVisitedGoodPoints():
        if (euclidianDistance(fromPoint, point) < distance):
            points.append(point)

    return points

def currentPositionPoint(self):
    return [self.x,self.y]

def euclidianDistance(self,A,B):
    A = np.array(A)
    B = np.array(B)
    return(np.sqrt(np.dot((A-B),(A-B))))

def degreeToRadians(degrees):
    return degrees * 0.17

def getRemainingTime(self):
    timeSpentInMs = time.time() - self.beginAt
    remainingTime = 120000 - timeSpentInMs

    if remainingTime < 0:
        remainingTime = 0

    return remainingTime

def remainingDistance(self):
    return(self.speed*(self.getRemainingTime()/1000))

'''
self.currentTarget = {}
self.goodPoints = []
self.badPoints = []
self.visitedGoodPoints = []
self.visitedBadPoints = []

self.beginAt = time.time()

self.speed = 49 # pt per sec
self.angularSpeed = 0.62 # radian/s
self.angularSpeedDegree = 36 # degree per sec

self.max_x = 1280
self.max_y = 960
'''
