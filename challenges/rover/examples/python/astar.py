import numpy as np
# Create a density map from point position

# Value function
V = time_to_go + (remaining_density * time_per_density)

# Model

# Density
def getDensityGrid():
    grid = np.zeros((10, 10))
    return grid

def getNumberOfPointInCell(point, cell):
    px = point[0]
    py = point[1]
    cellx = cell[0]
    celly =  cell[1]
    cellWidth = cell[2]
    cellHeight = cell[3]

    if ((py > celly) && (py <= celly + cellHeight)&&(px> cellx)&&(px =< cellx+cellWidth)):
        return True 

def getDensityForPoint():
    # either you place the point in the grid a get the value of the grid
    # or you compute the local density
    # density radius should depend on remaining time
    return density
