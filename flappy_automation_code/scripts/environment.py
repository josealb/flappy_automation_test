
import numpy as np
import math
import matplotlib.pyplot as plt

class openingEstimator:
    numberOfScanRays = 9
    openingProbability = [] #Probability that opening is at any of the 9 scan angles
    env_map = []
    ego_position = []
    goal_position = []

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
                #print("added point "+str(x)+"," ,str(y))
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
        if self.goal_position!=[]:
            with open('/home/flyatest/goal_position.txt','a+') as f:
                f.write(str(self.goal_position[0])+','+str(self.goal_position[1])+'\n')

    def updatePosition(self,velocity):
        self.ego_position[0] = self.ego_position[0] + velocity.x/30
        self.ego_position[1] = self.ego_position[1] + velocity.y/30
        print("Ego Position: "+str(self.ego_position))

    def getcollisionAvoidanceOutput(self):
        threshold = 0.3
        correction = [0,0]
        closestUpperObstacle = [100, 100]
        closestLowerObstacle = [-100, -100]
        smallestDistance = 999
        for i in range(0,len(self.env_map)):
            #used displacement instead of distance because distance cannot be negative
            #Find the most restrictive upper and lower boundaries in a 1m range around the bird
            
            if self.env_map[i][0]>self.ego_position[0]-1 and self.env_map[i][0]<self.ego_position[0]+1:
                #print("found obstacles in x range")
                y_displacement = self.env_map[i][1]-self.ego_position[1]
                if y_displacement > 0:
                    if abs(y_displacement)<abs(closestUpperObstacle[1]-self.ego_position[1]):
                        closestUpperObstacle = self.env_map[i]
                if y_displacement < 0 and self.env_map[i][1]<(closestUpperObstacle[1]-0.4):
                    if abs(y_displacement)<abs(closestLowerObstacle[1]-self.ego_position[1]):
                        closestLowerObstacle = self.env_map[i]
                x_displacement = self.env_map[i][0]-self.ego_position[0]
                y_displacement = self.env_map[i][1]-self.ego_position[1]
                distance = math.sqrt(math.pow(x_displacement,2)+math.pow(y_displacement,2))   
                if distance < smallestDistance:
                    smallestDistance=distance     
        if smallestDistance<threshold:
            #When we get close to the rocks, we want to be as far as possible
            self.goal_position=[0,0]
            self.goal_position[1] = (closestUpperObstacle[1]+closestLowerObstacle[1])/2
            self.goal_position[0] = self.ego_position[0]
            error_to_goal = self.goal_position[1]-self.ego_position[1]
            correction[1]=-error_to_goal*5 
            print("Goal position y: "+str(self.goal_position[1]))
            print("Error_to_goal"+str(error_to_goal))
        else:
            self.goal_position=[]
        print("closestUpperObstacle: "+str(closestUpperObstacle)+"closestLowerObstacle: "+str(closestLowerObstacle))
        print("Correcting with_ "+str(correction))
        return correction



