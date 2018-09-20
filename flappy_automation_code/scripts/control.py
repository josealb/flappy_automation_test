import numpy as np

class controller:
    maxProbability=0
    centerOfMass = 0
    previous_error = 0
    integral_error = 0
    measurements = []

    tau_p = 2
    tau_i = 0#0.001
    tau_d = 60

    y_coord_start = -1.4
    y_coord_end = 2.5
    number_of_increments = 15

    def updateMeasurements(self,newMeasurements):
        self.measurements = newMeasurements
    
    def getCenterOfMass(self,openingProbability):
        weighted_sum = 0
        sum = 0
        for i in range(0,len(openingProbability)):
            weighted_sum += openingProbability[i]*i
            sum += openingProbability[i]
        center = weighted_sum / sum
        increment = (abs(self.y_coord_start-self.y_coord_end))/self.number_of_increments #translate center of mass increment to coordinates
        self.centerOfMass = center*increment+self.y_coord_start
        print ("Center of mass is: "+str(center))

    def getMax(self,openingProbability):
        center = np.argmax(openingProbability)
        
        increment = (abs(self.y_coord_start-self.y_coord_end))/self.number_of_increments #translate center of mass increment to coordinates
        #self.centerOfMass = center*increment+self.y_coord_start
        self.maxProbability = center*increment+self.y_coord_start
        print ("Center of mass is: "+str(center))

    def PIDupdate(self,ego_position):
        #PID Error is center of mass since center
        error = self.maxProbability - ego_position[1]+0.15 #Bird should be centered on route with fewer obstacles
        #clipping the error
        if error > 0.4:
            error = 0.4
        if error < -0.4:
            error = -0.4

        diff_error = error - self.previous_error
        self.previous_error = error

        if diff_error > 0.1:
            diff_error = 0.1
        elif diff_error < -0.1:
            diff_error = -0.1    
        self.integral_error += error
        steer = self.tau_p * error + self.tau_d * diff_error + self.tau_i*self.integral_error
        print ("Error: "+str(error)+" Diff_error: " + str(diff_error)+"Integral error: " + str(self.integral_error))
        return steer

    def getControlUpdate(self, openingProbability,ego_position):
        #self.getCenterOfMass(openingProbability)
        self.getMax(openingProbability)
        steer = self.PIDupdate(ego_position)
        x=0
        return [x,steer]