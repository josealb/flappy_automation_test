
import numpy as np
import math
import matplotlib.pyplot as plt
class openingEstimator:
    numberOfScanRays = 9
    openingProbability = [] #Probability that opening is at any of the 9 scan angles
    env_map = []
    def __init__(self):
        self.openingProbability = np.zeros(self.numberOfScanRays)
    def updateMeasurements(self, measurements):
        for i in range(0,9):
            self.openingProbability[i] = self.openingProbability[i]+(measurements[i]-1.8)
            print("openingProbability at "+str(i)+"= "+str(self.openingProbability[i]))
    def accumulatePoints(self, measurements,angle_min,angle_increment):
        for i in range (0,len(measurements)):
            if measurements[i]<3.5:#means it hit something
                x = measurements[i]*math.cos(angle_min+i*angle_increment)
                y = measurements[i]*math.sin(angle_min+i*angle_increment)
                self.env_map.append([x, y])
                print("added point "+str(x)+"," ,str(y))
        #self.plotMap()
        self.saveMap()
    def plotMap(self):
        for i in range (0,len(self.env_map)):
            plt.scatter(self.env_map[i][0],self.env_map[i][1])
    def saveMap(self):
        with open('/home/flyatest/map.txt','w') as f:
            for i in range(0,len(self.env_map)):
                f.write(str(self.env_map[i][0])+','+str(self.env_map[i][1])+'\n')

