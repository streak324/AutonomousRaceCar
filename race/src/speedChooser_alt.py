#!/usr/bin/env python

#   Node: [(x,y),[UP=0, RIGHT=1, DOWN=2, LEFT=3]]
#       Directions are clockwise, starting from up


#-----          Imports Start                   -----#

from math import sqrt           #Square root for distance_finder
import rospy                    #ROS package for use in python, rospy
from math import *
from std_msgs.msg import Int32  #Standard messages Int32 to publish 'side'
from std_msgs.msg import Bool  #Standard messages Bool to publish 'is_turning'
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
#-----          Imports End                     -----#


brake_activated = False
speed_up_activated = False

#-----          Internal Functions Start        -----#


'''
Brake and then go at 20 velocity
'''
def brakePump():
        print("Starting BRAKE PUMP at "+str(car_x)+","+str(car_y))
        for i in range(1):
            v_msg = Int32(-70)
            for x in range(10):
                v_msg.data -= 10
                em_pub.publish(v_msg)

            time.sleep(0.4)

            for x in range(19):
                v_msg.data += 10 
                if v_msg.data > 12:
                    v_msg.data = 12
                em_pub.publish(v_msg)

            #time.sleep(0.3)
            
        print("BRAKES PUMPED at "+str(car_x)+","+str(car_y))

      
def distance_from_node():                                       #Output: distance of car from current node as an integer (negative value means it passed the current node)
    global currentNode, car_x, car_y,nodes                          #Global variables for node info and car position
    (x,y)=nodes[currentNode][0]                                     #Current node's position on map
    (x_next,y_next)=nodes[(currentNode+1)%len(nodes)][0]            #Next node (x,y) to see if car has passed the current node or not
    sign = -1 if distance((x,y),(x_next,y_next))>distance((car_x,car_y),(x_next,y_next)) else 1
                                                                    #Finds distance's sign; negative means car has passed node (car is closer to next node than current node is)
    return sign*distance((car_x,car_y),(x,y))                       #Integer distance from car to node

def distance(p1,p2):                                            #Functionality: Distance formula calculator
    return int(sqrt((p2[1]-p1[1])*(p2[1]-p1[1])+(p2[0]-p1[0])*(p2[0]-p1[0])))

#-----          Internal Functions End          -----#





#-----      Directly Usable Functions Start     -----#

def setSpeed(s):                                                 #Input: Integer 's' that is speed
                                                                #Functionality: Sets 'speed' variable to 's'
    msg = Int32()                                                   #Message of type Int32, which will be published
    msg.data = s                                                    #Sets message data to 's'
    #print("set speed to ",s) 
    em_pub.publish(msg)                                             #Publishes message to 'side' variable


def setTurning(turn):
	turn_pub.publish(turn)
	#print("Set 'turning' to ",turn)
	

def do_stuff():                                                 #Functionality: function called every time (x,y) is updated
    global brake_activated, speed_up_activated 
    global currentNode, nodes, in_threshold,out_threshold,out_threshold_turn,in_threshold_turn           #Global variables for node info and in/out thresholds
    current_dist=distance_from_node()                               #Distance from the current node (for checking thresholds)
    if (current_dist>=0):                                           #If 'entering' current node
        if (current_dist<in_threshold):                                 #If car has entered node's range
            #setSpeed(12)                                            #Publish speed
            pass
        if (current_dist<in_threshold_turn):
            if not brake_activated:
                brake_activated = True 
                speed_up_activated = False 
                brakePump()
                #in_threshold_turn += 3
                #print("Lag incrementing: in_threshold_turn at ", in_threshold_turn) 
                setTurning(True)
    elif (current_dist<0):                                          #If 'exiting' current node
        if (current_dist<-1*out_threshold):                             #If car has fully exited node's range
            currentNode=(currentNode+1)%len(nodes)                          #Update node to the next node
            #print("Set current node to ",currentNode)
            pass 
        if (current_dist<-1*out_threshold_turn):
            if not speed_up_activated:
                brake_activated = False 
                speed_up_activated = True 
                setSpeed(23)
                setTurning(False)
            #do_stuff()
 
#-----      Directly Usable Functions End       -----#





#-----          Changeable Variables Start      -----#

(car_x,car_y)=(0,0)                             # ******* car x and y should actually be subscribed to node that constantly updates x,y
currentNode=rospy.get_param("current_node",1)    #Start node; this number is the index of the node that the car is current on or reaching
direction=rospy.get_param("direction",1)         #A direction of 1 means the car is traveling clockwise, -1 is counterclockwise (later reverses nodelist if counterclockwise)
in_threshold=7                                 #Threshold for how close to the node you must be to set the wall
out_threshold=5                                 #Threshold for how far from the node you must reach to move on to next node
nodes=[ [(0.5,-.2),[0,1,1,0]], [(29,0),[0,0,1,1]], [(29.5,-17.9),[1,0,0,1]], [(1,-19),[1,1,0,0]] ]
                                                #Nodes list in order of traversal, nodes are in format [(x,y),[up=0,right=1,down=2,left=3]]
in_threshold_turn=14 
out_threshold_turn=0

#-----          Changeable Variables End        -----#


def callback(data):
    global car_x,car_y 
    x = floor(data.pose.pose.position.x)
    y = floor(data.pose.pose.position.y)
    if abs(x-car_x)>=1 or abs(y-car_y)>=1:
        print("X: ",car_x,", Y: ",car_y,"; Dist: ",str(distance_from_node()),", CurrNode: ",currentNode) 
        car_x = x
        car_y = y 
        do_stuff()  


#-----          Initialization Start            -----#
if __name__=='__main__':
    rospy.init_node('speed_control', anonymous=True)         #Create node to publish SIDE to ("side_control")
    em_pub = rospy.Publisher('drive_velocity', Int32, queue_size=1)   #Make the publisher for 'side' variable
    turn_pub = rospy.Publisher('is_turning', Bool, queue_size=1)   #Make the publisher for 'is_turning' variable
    
    if (direction==-1):                                     #Reverses order of traversing nodes if counterclockwise
        nodes.reverse()
        currentNode=len(nodes)-1-currentNode                #Adjust currentNode to compensate for flipped nodes list

    setSpeed(0)
    time.sleep(5)    
    setSpeed(23)          #Sets initial speed to 30
    sub = rospy.Subscriber('amcl_pose',PoseWithCovarianceStamped,callback) 
    rospy.spin()
#-----          Initialization End              -----#
