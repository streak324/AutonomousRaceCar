#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

location = [0,0,0]
prev = None
odom_pub = rospy.Publisher('odom_est', Odometry, queue_size=10)
VELOCITY = 0

def save_velocity(data):
    global VELOCITY
    VELOCITY = data.data

def predictor(data):
    global VELOCITY,prev
    #car's velocity, previous time stamp
    
    #car's velocity in each direction
    avel = data.angular_velocity
    dx = avel[0]
    dy = avel[1]
    dz = avel[2]
    
    dt = data.header.stamp - prev
    prev = data.header.stamp

    location[0] = dx/dt*VELOCITY
    location[1] = dy/dt*VELOCITY
    location[2] = dz/dt*VELOCITY

    msg = Odometry()
    msg.pose = PoseWithCovariance()
    msg.pose.pose = Pose()
    msg.pose.pose.position = Point()


if __name__=='__main__':
    rospy.init_node('odom', anonymous=True)
    rospy.Subscriber('/imu', Imu, predictor)
    rospy.Subscriber('/velocity', Float64, save_velocity)

    rospy.spin()
