import numpy as np
# Create a density map from point position

# Value function
V = time_to_go + (remaining_density * time_per_density)

# Model

# Density
def getDensityGrid(bins = 10):
    grid = np.zeros((10, 10))
    cellWidth = self.max_x / bins
    cellHeight = self.max_y / bins

    for x in range(0,bins):
        for y in range (0,bins):
            grid[x,y] = getNumberOfPointInCell()
    return grid

def pointsInCell (cell, onlyNonVisited = True):
    points = []
    for point in getNonVisitedGoodPoints():
        if (isPointInCell(point, cell):
            points.append(point)

    return points

def isPointInCell(point, cell):
    px = point[0]
    py = point[1]
    cellx = cell[0]
    celly =  cell[1]
    cellWidth = cell[2]
    cellHeight = cell[3]

    if ((py > celly) && (py <= celly + cellHeight)&&(px> cellx)&&(px =< cellx+cellWidth)):
        return True

def getDensityForPoint(totalPoints):
    # either you place the point in the grid a get the value of the grid
    # or you compute the local density
    # density radius should depend on remaining time
    remainingPoints = len(self.visitedGoodPoints)
    vicinityPoint = self.pointCloserThan(self.remainingDistance())

    return (len(vicinityPoint)/remainingPoints)

def pointCloserThan(distance, fromPoint = None):
    if fromPoint is None:
        fromPoint = self.currentPositionPoint()

    points = []
    for point in self.getNonVisitedGoodPoints():
        if (euclidianDistance(fromPoint, point) < distance):
            points.append(point)

    return points
