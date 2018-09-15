
import numpy as np
class openingEstimator:
    numberOfScanRays = 9
    openingProbability = [] #Probability that opening is at any of the 9 scan angles
    def __init__(self):
        self.openingProbability = np.zeros(self.numberOfScanRays)
    def updateMeasurements(self, measurements):
        for i in range(0,9):
            self.openingProbability[i] = self.openingProbability[i]+(1.8-measurements[i])
            print("openingProbability at "+str(i)+"= "+str(self.openingProbability[i]))

        return 0#TODO
