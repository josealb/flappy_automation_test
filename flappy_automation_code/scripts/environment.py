
import numpy as np
import math
import matplotlib.pyplot as plt

class openingEstimator:
    numberOfScanRays = 9
    openingProbability = [] #Probability that opening is at any of the 9 scan angles
    env_map = []
    ego_position = []

    def __init__(self):
        self.openingProbability = np.zeros(self.numberOfScanRays)
        self.ego_position = [0,0]
        open('/home/flyatest/ego_position.txt','w').close()

    def updateMeasurements(self, measurements):
        for i in range(0,9):
            self.openingProbability[i] = self.openingProbability[i]+(measurements[i]-1.8)
            print("openingProbability at "+str(i)+"= "+str(self.openingProbability[i]))

    def accumulatePoints(self, measurements,angle_min,angle_increment):
        max_map_size = 1500
        while len(self.env_map)>max_map_size:
            self.env_map.pop(0)
        for i in range (0,len(measurements)):
            if measurements[i]<3.5:#means it hit something
                x = measurements[i]*math.cos(angle_min+i*angle_increment)+self.ego_position[0]
                y = measurements[i]*math.sin(angle_min+i*angle_increment)+self.ego_position[1]
                self.env_map.append([x, y])
                print("added point "+str(x)+"," ,str(y))
        self.saveMap()

    def plotMap(self):
        for i in range (0,len(self.env_map)):
            plt.scatter(self.env_map[i][0],self.env_map[i][1])

    def saveMap(self):
        with open('/home/flyatest/map.txt','w') as f:
            for i in range(0,len(self.env_map)):
                f.write(str(self.env_map[i][0])+','+str(self.env_map[i][1])+'\n')
        with open('/home/flyatest/ego_position.txt','a+') as f:
                f.write(str(self.ego_position[0])+','+str(self.ego_position[1])+'\n')

    def updatePosition(self,velocity):
        self.ego_position[0] = self.ego_position[0] + velocity.x/30
        self.ego_position[1] = self.ego_position[1] + velocity.y/30
        print("Ego Position: "+str(self.ego_position))

    def getcollisionAvoidanceOutput(self):
        closestObjectDistance = 999
        threshold = 0.2
        correction = [0,0]
        for i in range(0,len(self.env_map)):
            #used displacement instead of distance because distance cannot be negative
            if self.env_map[0]>self.ego_position[0]-1:
                x_displacement = self.env_map[i][0]-self.ego_position[0]
                y_displacement = self.env_map[i][1]-self.ego_position[1]
                distance = math.sqrt(math.pow(x_displacement,2)+math.pow(y_displacement,2))
                if distance<threshold:
                    #nudge bird in the direction oposite to the obstacle, with more weight for closer obstacles
                    #correction[0]+=-x_displacement * (threshold-abs(x_displacement)/threshold)
                    
                    #correction[0]=0 #no horizontal correction
                    correction[1]+=-y_displacement * (threshold-abs(y_displacement)/threshold)*0.6
        print("Correcting with_ "+str(correction))
        return correction



