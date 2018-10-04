#!/usr/bin/env python

import Tkinter as tk
import rospy
from race.msg import drive_param
from race.msg import pid_input
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import math

#import constants

# (kd,SCALE_FACTOR)
#constants = {'30': (12.5,1.5), '12': (.09,1), '45': (15,1.5)}

def average(array):
    total = 0
    for value in array:
        total += value
    return total / len(array)

def dist_between(pos1, pos2):
    return math.sqrt( (pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 )

class RacecarAI:
    def __init__(self, car, screen): #original code: def__init__(self, car:"Racecar", screen):
        self.state = "auto"
        self.car = car
        # self.codriver = Codriver(car) # in ROS, Codriver will just be a seperate node
        self.fricCoeff = 1 # this will have to be adjusted to a realistic value
        self.safetyMode = False
        self.collisionAvoid = False
        self.screen = screen # not necessary, just here to display vector to longest distance
        self.start_angle = self.car._angle # used to keep track of how much the car has turned at nodes, updated in autoProgram
        self.start_pos = self.car._position # used to record distance traveled by car during collision avoidance mode

    def moveTowardsLongestDist(self, scope, right_bound, left_bound):
        ''' Uses the lidar to find the angle with the longest distance reading, and returns that angle. The angle is
        relative to the direction the car is facing. 0 degrees should be straight forward. Left angles are negative and
        right angles are positive.
        scope - limits the band of view which that car is using to make its decisions
        right_bound, left_bound - bottom and top boundary on the lidar indices that are taken into consideration 
        '''
        angle = math.pi / len(self.car.lidar)
        maximum = 0
        max_index = 0
        for reading in range(right_bound+scope, left_bound-scope):
            if(self.car.lidar[reading] >= maximum):
                maximum = self.car.lidar[reading]
                max_index = reading
            
        new_angle = angle*max_index - math.pi/2 # subtract PI/2 to make angle=0 -> angle=-PI/2

        #draw line just for gui visual
        self.screen.create_line(self.car._position[0], self.car._position[1],
                               self.car._position[0] + math.cos(new_angle + self.car._angle)*50,
                               self.car._position[1] + math.sin(new_angle + self.car._angle)*50, fill="#00FFFF")

        return new_angle

    def autoProgram(self):
        #CHECK FOR UNAVOIDABLE CRASH
        ''' If the distance reading directly in front of the car is less that the minimum turning radius
            given velocity of the car and friction, car cannot turn to avoid the crash
        '''
        middle = len(self.car.lidar)//2
        front_dist = 0
        for i in range(-1,2):
            front_dist += (self.car.lidar[middle+i])
        front_dist = front_dist / 3
        
        if(self.car.velocity**2/(front_dist/3) > 9.81*self.fricCoeff):
            self.safetyMode = True
            print("safety mode on")

        #CHECK FOR OBSTACLE
        if(front_dist < 100):
            self.collisionAvoid = True
        
        #SET CAR VELOCITY PREPORTIONAL TO FRONT DISTANCE
        motor_speed = int(15 + 30*(front_dist/500))
        self.car.changeMotorSpeed(motor_speed)

        #DETERMINE ANGLE TO TURN WHEELS TO
        right = self.car.lidar[0]
        left = self.car.lidar[len(self.car.lidar)-1]
        angle = math.pi / len(self.car.lidar)
        left_slopes = []
        right_slopes = []
        # average of slopes between left and right lidar readings
        for i in range(5):
            left_x1 = self.car.lidar[1] * math.cos(angle)
            left_y1 = self.car.lidar[1] * math.sin(angle)
            left_x2 = self.car.lidar[2] * math.cos(2*angle)
            left_y2 = self.car.lidar[2] * math.sin(2*angle)
            left_slopes.append( math.atan2((left_y2-left_y1) , (left_x2-left_x1)) )
            right_x1 = self.car.lidar[-2] * math.cos(math.pi - angle)
            right_y1 = self.car.lidar[-2] * math.sin(math.pi - angle)
            right_x2 = self.car.lidar[-3] * math.cos(math.pi - 2*angle)
            right_y2 = self.car.lidar[-3] * math.sin(math.pi - 2*angle)
            right_slopes.append( math.atan2((right_y2-right_y1) , (right_x2-right_x1)) )

        # if in a straight hallway type path
        ''' If the angle of left and right wall are kinda the same'''
        if(abs(average(left_slopes) - average(right_slopes)) < math.pi/10):
            new_angle = (average(left_slopes) + average(right_slopes) - math.pi)/2
        else: # if not in a straight hallway type path...
            new_angle = self.moveTowardsLongestDist(0, 0, len(self.car.lidar))
                
        #if car is too close to a wall, modify angle to pull away
        if(right < 45):
            #print("too close to right")
            new_angle += math.pi/12
        if(left < 45):
            #print("too close to left")
            new_angle -= math.pi/12
                
        # turn the wheels of the car according to the new_angle    
        ''' The intensity of angle change is preportional to the difference in current angle and desired angle'''
        diff = abs(new_angle - self.car.turnAngle)
        if(new_angle > self.car.turnAngle): # right
            self.car.changeTurnAngle(self.car.turnAngle + 0.2*diff)
        elif(new_angle < self.car.turnAngle): # left
            self.car.changeTurnAngle(self.car.turnAngle - 0.2*diff)

        # update the start angle for decision nodes
        ''' During a turn, the start_angle is not updated, and is used a reference to measure how much the car has turned'''
        self.start_angle = self.car._angle
        self.start_pos = self.car._position

    def slowProgram(self):
        pass

    def avoidCollision(self):
        # COLLSISION AVOIDANCE
        motor_speed = 30
        self.car.changeMotorSpeed(motor_speed)
        new_angle = self.moveTowardsLongestDist(0, 0, len(self.car.lidar))

        #if(new_angle > 0):
        #    new_angle += math.pi/18
        #elif(new_angle < 0):
        #    new_angle -= math.pi/18
        
        if(new_angle > self.car.turnAngle): # right
            self.car.changeTurnAngle(self.car.turnAngle + 0.2)#weight*diff)
        elif(new_angle < self.car.turnAngle): # left
            self.car.changeTurnAngle(self.car.turnAngle - 0.2)#weight*diff)

        dist = dist_between(self.start_pos, self.car._position)
        #print(dist)
        #print(self.start_pos)
        if(dist > 80):
            self.collisionAvoid = False

    def turningProgram(self):
        #set car velocity preportional to front dist
        #front_dist = self.car.lidar[len(self.car.lidar)//2]
        motor_speed = 30
        self.car.changeMotorSpeed(motor_speed)
        
        #decide turning angle
        ''' This makes sure Driver does not turn too much at a node. Planned to make the turning cap 90 degress, but found
        that an underestimate like 30 degrees work a lot better. Turing program only has to nudge the drive toward
        the right direction, after that, the autoProgram can stabalize the car on its path much better.'''
        if(abs(self.start_angle - self.car._angle) <= math.pi/6): # if car hasnt turned 30 degrees yet
            if(self.state == "left"):
                #print("left")
                right = 0
                left = len(self.car.lidar)//2-1
                new_angle = self.moveTowardsLongestDist(0, right, left)
            elif(self.state == "right"):
                #print("right")
                right = len(self.car.lidar)//2+1
                left = len(self.car.lidar)
                new_angle = self.moveTowardsLongestDist(0, right, left)
        else:
            #print("TURNED ENOUGH")
            new_angle = self.moveTowardsLongestDist(15, 0, len(self.car.lidar))
            
        diff = abs(new_angle - self.car.turnAngle)
        if(new_angle > self.car.turnAngle): # right
            self.car.changeTurnAngle(self.car.turnAngle + 0.1)#*abs(max_index - self.car.reading_number//2))
        elif(new_angle < self.car.turnAngle): # left
            self.car.changeTurnAngle(self.car.turnAngle - 0.1)#*abs(max_index - self.car.reading_number//2))

    def safetyProgram(self):
        if(self.car.velocity > 0.1): #not stopped
            self.car.changeMotorSpeed(-30)
        self.car.changeMotorSpeed(0)

    def main_funct(self):
        # check for a changed state from Codriver
        self.state = self.codriver.chooseState()
        
        if(not self.safetyMode):
            if(self.collisionAvoid):
                print("COLLISION AVOID")
                self.avoidCollision()
            elif(self.state == "auto"):
                print("auto")
                self.autoProgram()
            elif(self.state == "left" or self.state == "right"):
                print(self.state)
                self.turningProgram()
            elif(self.state =="slow"):
                self.slowProgram()
        else:
            self.safetyProgram()

if __name__ == '__main__':
    print("Listening to error for PID")
    rospy.init_node('pid_controller', anonymous=True)
    rospy.Subscriber('side',Int32, updateSide)
    rospy.Subscriber("error", pid_input, control)
    rospy.Subscriber("drive_velocity", Int32, updateVelocity) 
    rospy.spin()
