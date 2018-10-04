#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu 

pub = rospy.Publisher('imu/data', Imu, queue_size=1)
avg = 0
N = 0
BUFFER = [] 

def fix(data):
    global avg, N
    data.linear_acceleration.x -= 0#1.35222529899
    data.linear_acceleration.y += -0.014
    data.linear_acceleration.z += 1.054
    pub.publish(data)

    if avg == 0: 
         avg = data.linear_acceleration.x
         N = 1

    BUFFER.append(data.linear_acceleration.x)
    approxRollingAverage(data.linear_acceleration.x)

def approxRollingAverage(new_sample):
       global avg, N
       avg = avg - avg/N
       avg = avg + new_sample/N
       N = N + 1
       print(avg)


if __name__ == '__main__':
            print("Doing stuff to Imu")
            rospy.init_node('imu_fixer', anonymous=True)
            rospy.Subscriber("imu", Imu, fix)
            rospy.spin()


