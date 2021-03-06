#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import environment as en
import control as con
from copy import copy
# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)
estimator = en.openingEstimator()
controller = con.controller()

def initNode():
    # Here we initialize our node running the automation code
    rospy.init_node('flappy_automation_code', anonymous=True)

    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    rospy.Subscriber("/flappy_vel", Vector3, velCallback)
    rospy.Subscriber("/flappy_laser_scan", LaserScan, laserScanCallback)

    # Ros spin to prevent program from exiting
    rospy.spin()

def velCallback(msg):
    # msg has the format of geometry_msgs::Vector3
    # Example of publishing acceleration command on velocity velCallback
    x = 0   
    y = 0
    estimator.updatePosition(msg)
    [openingProbability,ego_position] = estimator.findOpening()
    [x,y] = controller.getControlUpdate(openingProbability,ego_position)
    [x_avoidance,y_avoidance] = estimator.getcollisionAvoidanceOutput()
    if x_avoidance!=0 or y_avoidance!=0:
        x=-x_avoidance
        y=-y_avoidance
        print("Avoiding obstacle")
    
    print("control output x: "+str(x)+" y: "+str(y))
    pub_acc_cmd.publish(Vector3(x,y,0))

def laserScanCallback(msg):
    # msg has the format of sensor_msgs::LaserScan
    # print laser angle and range
    #print "Laser range: {}, angle: {}".format(msg.ranges[0], msg.angle_min)
    estimator.accumulatePoints(copy(msg.ranges),msg.angle_min,msg.angle_increment)
    

if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
