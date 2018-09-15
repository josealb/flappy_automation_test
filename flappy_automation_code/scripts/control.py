import numpy as np
class controller:
    centerOfMass = 0
    previous_error = 0
    integral_error = 0
    measurements = []

    tau_p = 0.5
    tau_i = 0
    tau_d = 20

    def updateMeasurements(self,newMeasurements):
        self.measurements = newMeasurements
    
    def getCenterOfMass(self):
        weighted_sum = 0
        sum = 0
        for i in range(0,len(self.measurements)):
            weighted_sum += self.measurements[i]*i
            sum += self.measurements[i]
        center = weighted_sum / sum
        self.centerOfMass = center
        print ("Center of mass is: "+str(center))

    def PIDupdate(self):
        #PID Error is center of mass since center
        error = self.centerOfMass - 4 #Bird should be centered on route with fewer obstacles
        diff_error = error - self.previous_error
        self.previous_error = error
        self.integral_error += error
        steer = self.tau_p * error + self.tau_d * diff_error + self.tau_i*self.integral_error
        print ("Error: "+str(error)+" Diff_error: " + str(diff_error)+"Integral error: " + str(self.integral_error))
        return steer

    def getControlUpdate(self):
        self.getCenterOfMass()
        steer = self.PIDupdate()
        x=0
        return [x,steer]