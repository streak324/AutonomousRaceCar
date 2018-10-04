#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

em_pub = rospy.Publisher('velocity', Float64, queue_size=1)

# imu update rate in hertz
DELTA_T = 20
THRESH_X = 0.3
THRESH_Y = 0.005
VELOCITY = Float64(0)

SIGN = 1

prev = None

def sign_callback(data):
        global SIGN
        if previous_velocity==data.data:
                pass
        else:
                if previous_velocity>data.data:
                    SIGN = -1
                else:
                    SIGN = 1
                previous_velocity = data.data

def callback(data):
        global VELOCITY, prev

        dt = 0

        if prev==None:
            prev = data.header.stamp.to_sec() 
        else:
            dt = data.header.stamp.to_sec() - prev
            prev = data.header.stamp.to_sec() 


        # calculate velocity according to delta time and vf = a*t
        # this is basically integration of acceleration and may not be accurate but worth a try
        if not (data.linear_acceleration.x <= THRESH_X and data.linear_acceleration >= 0):
            VELOCITY.data += SIGN*round(data.linear_acceleration.x,1)*dt #(1.0/48) 
            print(str(data.header.stamp.to_sec()) + ", "+str(data.linear_acceleration.x)) 
        em_pub.publish(VELOCITY) 

            #VELOCITY.data += data.linear_acceleration.x*(1.0/(20*math.pow(10,6)))


        #em_pub.publish(VELOCITY)
            


        
if __name__ == '__main__':
        print("Velocity detector started")
        rospy.init_node('velocity_detector',anonymous=True)
        rospy.Subscriber('imu/data',Imu, callback)
        rospy.Subscriber('drive_velocity', Int32, sign_callback)
        rospy.spin()

