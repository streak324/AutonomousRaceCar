#!/usr/bin/env python

import constants 
import rospy
from std_msgs.msg import Bool
from race.msg import drive_param
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
'''Need rospy and message types for eStop (bool), drive_parameters (drive_param), scan (LaserScan)'''

from math import radians, degrees, pi #for conversions
from math import * #for conversions

from numpy import isnan
import numpy 


#global parameters
'''Drive parameters'''
velocity = rospy.get_param("/initial_speed", "12")
SIDE = rospy.get_param("/initial_side", -1)
#SIDE = 1 is right SIDE = -1 is left

'''Publisher'''
#em_pub = rospy.Publisher('eStop', Bool, queue_size=10)
v_pub  = rospy.Publisher('drive_velocity', Int32, queue_size=1)
activated = False


# 12 --> 1
# 30 --> 2
# 45 --> 5.5
### 5.5 is good enough but 6 for extra safety

FRONT_BUMPER_THRESHOLD = 3.0 
SAFETY_MODE = True
WIDTH = 1.2 #.3


'''Update saved drive parameters'''
def save_drive(drive_data):
    global velocity
    velocity = drive_data.velocity
    set_threshold()

'''
Brake and then go at 20 velocity

def brakePump():
    global em_pub, activated
    v_msg = Int32(-70)
    for x in range(10):
        v_msg.data -= 10
        v_pub.publish(v_msg)

    for x in range(19):
        v_msg.data += 10
        v_pub.publish(v_msg)

    #em_pub.publish(True)
    print("BREAKS PUMPED")
'''
use_camera = rospy.get_param("/use_camera", "false")
camera_sees_wall = False

'''If camera sees side wall, ignores opinion that about to crash'''
def camera_callback(cam_decision):
    global camera_sees_wall
    camera_sees_wall = cam_decision.data
    #camera_sees_wall = False

is_not_wall = lambda : not camera_sees_wall

'''
Check for immediate obstacles that we are not avoiding
If the data points around our headed trajectory seem close,
pull emergency stop, which can be reset in kill switch
'''
def safety_checker(laser_data):
    global em_pub, activated
    v_msg = Int32(-70)
    if not activated and detect_collision(laser_data) and is_not_wall():
        v_pub.publish(v_msg) 
        activated = True

        
        
        for x in range(10):
            v_msg.data -= 10
            v_pub.publish(v_msg)

        
        #em_pub.publish(True)
        print("Emergency STOP!!!!!")

'''
Input:  data: Lidar scan data
        theta_start: Min angle data to give (RADIANS)
        theta_end: Max angle data to give (RADIANS)
OUTPUT: distance of scan at angle theta
'''
def getSomeScans(data,theta_start,theta_end):
    if (theta_start > theta_end):
        return None
    theta_0 = data.angle_min
    theta_delta = data.angle_increment
    
    start = int((theta_start-theta_0)/theta_delta)
    end = int((theta_end-theta_0)/theta_delta)

    subset = data.ranges[start:end]
    MIN = data.range_min
    return [x for x in subset if (~isnan(x) and x>MIN)] #discards garbage

##  Input:  data: Lidar scan data
##          theta: The angle at which the distance is requried
##  OUTPUT: distance of scan at angle theta
def getRange(data,theta):
# Find the index of the arary that corresponds to angle theta.
# Return the lidar scan value at that index
# Do some error checking for NaN and ubsurd values
## Your code goes here

    theta_0 = data.angle_min
    theta_delta = data.angle_increment

    idx = int((theta-theta_0)/theta_delta)

    i = 1

    for j in range(len(data.ranges)):
            # discard any values that are outside valid range
            if isnan(data.ranges[idx]) or data.ranges[idx] > data.range_max or data.ranges[idx] < data.range_min:
                    if (i % 2 == 0):
                            idx = idx + -i
                    else: 
                            idx = idx + i

                    i = i + 1
            else: 
                    return data.ranges[idx]

    return None


'''Check points around headed angle, and also right in front'''
def detect_collision(laser_data):
    global velocity,SIDE,FRONT_BUMPER_THRESHOLD



    theta = 50
    swing = radians(theta)
    a = getRange(laser_data,SIDE * (-pi/2+swing))
    b = getRange(laser_data,SIDE * (-pi/2))

    angle = abs(atan((a*cos(swing)-b)/(a*sin(swing))))

    if (angle > asin(WIDTH/2/FRONT_BUMPER_THRESHOLD)):
        threshold = WIDTH/2/sin(angle)
    else:
        threshold = FRONT_BUMPER_THRESHOLD

    left_theta = - asin(0.15/threshold)
    right_theta = - left_theta

    #left_theta = -pi/64
    #right_theta = pi/64

    #Check scans in immediate front
    distances = getSomeScans(laser_data,left_theta,right_theta)
    within_thresh = [x < threshold for x in distances]
    if numpy.mean(within_thresh) > .35:
        return True
    '''if min(distances)<threshold:
                    return True
    '
    #Check scans in trajectory
    distances = getSomeScans(laser_data,theta_d+left_theta,theta_d+right_theta)
    if min(distances)<FRONT_BUMPER_THRESHOLD:
        return True '''

    return False

'''If turning, checks less far ahead --> not used'''
def turning_callback(data):
    global FRONT_BUMPER_THRESHOLD
    FRONT_BUMPER_THRESHOLD = (.5 if data.data else 1.5)

'''Set side locally'''
def side_callback(data):
    global SIDE
    SIDE = data.data

'''Set threshold based on velocity'''
def set_threshold():
    global FRONT_BUMPER_THRESHOLD
    if (velocity in constants.PID_CONST.keys()):
        FRONT_BUMPER_THRESHOLD = constants.PID_CONST[velocity]['STOPPING_DISTANCE']
    #if SAFETY_MODE:
    #        FRONT_BUMPER_THRESHOLD += .5

if __name__=='__main__':
    rospy.init_node('wall_detector', anonymous=True)

    drive_sub = rospy.Subscriber('drive_parameters', drive_param, save_drive)
    laser_sub = rospy.Subscriber('scan', LaserScan, safety_checker)
    rospy.Subscriber('side',Int32,side_callback)
    #turn_sub = rospy.Subscriber('is_turning', Bool, set_threshold)

    if use_camera==True: #.lower()=="true":
        cam_sub = rospy.Subscriber('sees_side_wall',Bool,camera_callback)


    rospy.spin()

