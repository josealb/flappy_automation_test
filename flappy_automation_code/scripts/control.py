import numpy as np
class controller:
    centerOfMass = 0
    def getControlUpdate(self, openingProbability):
        '''
        index = np.argmax(openingProbability)
        print ("Most likely opening is in ray"+ str(index))
        '''
        x = 0
        y = 0
        if self.centerOfMass > 4:
            y = 1
        else:
            y = -1
        return [x,y]
        
    
    def getCenterOfMass(self, measurements):
        weighted_sum = 0
        sum = 0
        for i in range(0,len(measurements)):
            weighted_sum += measurements[i]*i
            sum += measurements[i]
        center = weighted_sum / sum
        self.centerOfMass = center
        print ("Center of mass is: "+str(center))
