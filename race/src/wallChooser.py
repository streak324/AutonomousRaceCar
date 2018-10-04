#!/usr/bin/env python

#   Node: [(x,y),[UP=0, RIGHT=1, DOWN=2, LEFT=3]]
#       Directions are clockwise, starting from up


#-----          Imports Start                   -----#

from math import *           #Square root for distance_finder
import rospy                    #ROS package for use in python, rospy
from std_msgs.msg import Int32  #Standard messages Int32 to publish 'side'
from geometry_msgs.msg import PoseWithCovarianceStamped
#-----          Imports End                     -----#

side_published = False



#-----          Internal Functions Start        -----#

def calculateSide(in_dir,out_dir,current_node_index):        
    #Input: 'in', 'out' directions (0,1,2,3) and current node index (from list 'nodes')
    #Output: which side to follow (-1 for left, 1 for right)
    global object                                                   #Global variable for nodes list
    if (abs(in_dir-out_dir)==2):                                    #If car will go straight through node ('in' and 'out' are opposite to each other)
        if (nodes[current_node_index][1][(in_dir-1)%4]==1):             #If a right turn is possible
            return -1                                                       #Follow left wall
        elif (nodes[current_node_index][1][(in_dir+1)%4]==1):           #If a left turn is possible
            return 1                                                        #Follow right wall
    else:                                                           #Else car will turn at this node
        if ((in_dir-out_dir)%4==1):                                     #If turn is a right turn (in is 1 more than out)
            return 1                                                        #Follow right wall
        else:                                                           #Else left turn
            return -1                                                       #Follow left wall


def findOut(current_node_index):                                #Input: Current node index (from list 'nodes')
                                                                #Output: out direction (0,1,2,3)
    global nodes                                                    #Global variable for nodes list
    p1=nodes[current_node_index][0]                                 #(x,y) position for current node
    p2=nodes[(current_node_index+1)%len(nodes)][0]                  #(x,y) position for next node
    return giveDirection(p1,p2)                                     #compute direction 


def findIn(carPosition, current_node_index):                    #Input: (x,y) tuple for the car, and the index of the incoming node (from list 'nodes')
                                                                #Output: direction the car will enter the node (0,1,2,3)
    global nodes                                                    #Global variable for nodes list
    p1=nodes[(current_node_index-1)%len(nodes)][0]                  #(x,y) position for previous node
    p2=nodes[current_node_index][0]                                 #(x,y) position for current node
    return giveDirection(p2, p1)                                    #Direction from the node to car
                                                                    #   Note: (p1,p2) will output the opposite direction since we want it relative to the current node

    
def giveDirection(p1,p2):                                       #Input: p1=(x1,y1) and p2=(x2,y2)
                                                                #Output: Direction exiting p1 if going to p2 (0,1,2,3)
    dx=p2[0]-p1[0]                                                  #Change in x values (x2-x1)
    dy=p2[1]-p1[1]                                                  #Change in y values (y2-y1)
    
    if (abs(dx)>abs(dy)):                                           #If traveling in the x direction (assuming either x or y is relevant, not both)
        if (dx>0):                                                      #If moving in positive x direction
            return 1                                                        #Output 'right'
        else:                                                           #If moving in negative x direction
            return 3                                                        #Output 'left'
    else:                                                           #If traveling in the y direction
        if (dy>0):                                                      #If moving in positive y direction
            return 0                                                        #Output 'up'
        else:                                                           #If moving in negative y direction
            return 2                                                        #Output 'down'

        
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

def setSide(s):                                                 #Input: Integer 's' that is 1 for right, -1 for left
                                                                #Functionality: Sets 'side' variable to 's'
    msg = Int32()                                                   #Message of type Int32, which will be published
    msg.data = s                                                    #Sets message data to 's'
    print("set side to "+str(s)+" at "+str(car_x)+", "+str(car_y)) 
    em_pub.publish(msg)                                             #Publishes message to 'side' variable

def findSide():                                                 #Functionality: generates current side to follow using earlier functions
    global currentNode, car_x, car_y , s                              #Global Variables needed to call inner functions
    return calculateSide(findIn((car_x,car_y),currentNode),findOut(currentNode),currentNode)
                                                                    #Calculate needed side by calling in/out functions on current node

def do_stuff():                                                 #Functionality: function called every time (x,y) is updated
    global side_published 
    global currentNode, nodes, in_threshold,out_threshold           #Global variables for node info and in/out thresholds
    current_dist=distance_from_node()                               #Distance from the current node (for checking thresholds)
    if (current_dist>=0):                                           #If 'entering' current node
        if (current_dist<in_threshold):                                 #If car has entered node's range
            if not side_published: 
                side_published = True
                setSide(findSide())                                            #Publish side
    elif (current_dist<0):                                          #If 'exiting' current node
        if (current_dist<-1*out_threshold):                             #If car has fully exited node's range
            side_published = False            
            currentNode=(currentNode+1)%len(nodes)                          #Update node to the next node
            #print("Set current node to ",currentNode)
            do_stuff()
 
#-----      Directly Usable Functions End       -----#





#-----          Changeable Variables Start      -----#

(car_x,car_y)=(0,0)                             # ******* car x and y should actually be subscribed to node that constantly updates x,y
currentNode=rospy.get_param("current_node",1)    #Start node; this number is the index of the node that the car is current on or reaching
direction=rospy.get_param("direction",1)         #A direction of 1 means the car is traveling clockwise, -1 is counterclockwise (later reverses nodelist if counterclockwise)
in_threshold=10                                 #Threshold for how close to the node you must be to set the wall
out_threshold=1                                 #Threshold for how far from the node you must reach to move on to next node
nodes=[ [(0.5,-.2),[0,1,1,0]], [(21,0),[0,1,1,1]], [(29,0),[0,0,1,1]], [(29.5,-17.9),[1,0,0,1]], [(1,-19),[1,1,0,0]] ]
                                                #Nodes list in order of traversal, nodes are in format [(x,y),[up=0,right=1,down=2,left=3]]

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
    rospy.init_node('side_control', anonymous=True)         #Create node to publish SIDE to ("side_control")
    em_pub = rospy.Publisher('side', Int32, queue_size=1)   #Make the publisher for 'side' variable
    
    if (direction==-1):                                     #Reverses order of traversing nodes if counterclockwise
        nodes.reverse()
        currentNode=len(nodes)-1-currentNode                #Adjust currentNode to compensate for flipped nodes list
    
    setSide(rospy.get_param("/initial_side", "-1"))          #Sets initial side (wall following) to left wall
    sub = rospy.Subscriber('amcl_pose',PoseWithCovarianceStamped,callback) 
    rospy.spin()
#-----          Initialization End              -----#
