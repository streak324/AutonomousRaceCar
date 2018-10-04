# racecar AI for driver

import math
import python_racecar

def average(array):
    total = 0
    for value in array:
        total += value
    return total / len(array)

def dist_between(pos1, pos2):
    return math.sqrt( (pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 )


class Codriver:

    def __init__(self, car:"Racecar"):
        self.nodes = [(85,85,"right"),(400,85,"right"),(400,515,"left"),
                      (715,515,"left"),(715,85,"left"),(400,85,"left"),
                      (400,515,"right"),(85,515,"right")]
        self.car = car
        self.next_index = 0
        self.at_node = False

    def chooseState(self):
        node = self.nodes[self.next_index]
        #if car is within a certain radius of node...
        if(math.sqrt((self.car._position[0]-node[0])**2 + (self.car._position[1]-node[1])**2) < 90):
            self.at_node = True
            return self.nodes[self.next_index][2]
        else:
            #choose next node and assign Driver a mode
            if(self.at_node): # exiting a node
                self.next_index += 1
                if(self.next_index >= len(self.nodes)):
                    self.next_index = 0
                self.at_node = False
            return "auto"

class RacecarAI:
    '''This is the Driver module that controls the car's basic functions
    It is designed to operate with or without a codriver.
    Without a codriver, the driver will simply drive slowly and safely, avoiding
    obstances and performing turns when necessary. It is designed to be lightweight
    and free of hardcoding.
    '''

    def __init__(self, car:"Racecar", screen, codriver, dumb):
        # codriver - bool that enables codriver
        # dumb - bool that indicates that driver should only avoid collisions
        self.dumb = dumb
        self.state = "auto"
        self.car = car
        if codriver == False:
            self.codriver = None
        else:
            self.codriver = Codriver(car) # in ROS, Codriver will just be a seperate node
        self.fricCoeff = 10 # this will have to be adjusted to a realistic value
        self.safetyMode = False
        
        ''' Variables for collision avoidance'''
        self.collisionAvoid = False
        self.obsDist = 0
        self.obsAngle = 0
        
        self.screen = screen # not necessary, just here to display vector to longest distance
        self.start_angle = self.car._angle # used to keep track of how much the car has turned at nodes, updated in autoProgram
        self.start_pos = self.car._position # used to record distance traveled by car during collision avoidance mode

    #-------------------------------------------------------------------------------------

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

    #-------------------------------------------------------------------------------------

    def _moveAwayFromWall(self, dist, angle):
        #if car is too close to a wall, modify angle to pull away
        right = self.car.lidar[0]
        left = self.car.lidar[len(self.car.lidar)-1]
        # dist will be scaled by number of carLengths (currently 1x)
        if(right < dist):
            #print("too close to right")
            return angle + math.pi/12
        if(left < dist):
            #print("too close to left")
            return angle - math.pi/12
        return angle

    def _getFrontDist(self, angle=0):
        middle = len(self.car.lidar)//2
        # relative angle ranges from -pi/2 to pi/2
        lidar_beam_angle = (math.pi / len(self.car.lidar))
        #determine the range of values to scan
        scope = int(math.atan(1/5) / lidar_beam_angle) + 1
        # this converts relative angle to corresponding LIDAR index
        index = int(angle / lidar_beam_angle + middle)
        if index + scope > len(self.car.lidar):
            index -= scope
        elif index - scope < 0:
            index += scope
        
        front_dist = self.car.lidar[index]
        for i in range(-scope,scope+1):
            if(self.car.lidar[index+i] < front_dist):
                front_dist = self.car.lidar[index+i]
        return front_dist

    def detectObstacle(self, front_dist):
        #CHECK FOR OBSTACLE within 4x current motor speed or 5x car lengths, whichever is greater
        if self.car.motorSpeed*4 < self.car.carLength*5:
            lookaheadDistance = self.car.carLength*5
            #print("lookahead by car length "+str(lookaheadDistance))
        else:
            lookaheadDistance = self.car.motorSpeed*4
            #print("lookahead by motor speed "+str(lookaheadDistance))
        if front_dist < lookaheadDistance:
            self.obsDist = front_dist
            return True
        else:
            return False
            
    def autoProgram(self):
        #CHECK FOR UNAVOIDABLE CRASH
        ''' If the distance reading directly in front of the car is less that the minimum turning radius
            given velocity of the car and friction, car cannot turn to avoid the crash
        '''
        front_dist = self._getFrontDist()
        
        if(self.car.velocity**2/(front_dist/3) > 9.81*self.fricCoeff):
            self.safetyMode = True
            print("safety mode on")
        self.collisionAvoid = self.detectObstacle(front_dist)
        if self.dumb:
            self.collisionAvoid = True
        
        #SET CAR VELOCITY PROPORTIONAL TO FRONT DISTANCE
        motor_speed = int(15 + 50*(front_dist/500))
        self.car.changeMotorSpeed(motor_speed)

        #DETERMINE ANGLE TO TURN WHEELS TO
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
            # intelligent auto driver behavior (the thing to improve)
            #new_angle = self.moveTowardsLongestDist(0, 0, len(self.car.lidar))
            new_angle = 0

        #if car is too close to a wall, modify angle to pull away
        new_angle = self._moveAwayFromWall(self.car.carLength, new_angle)
                
        # turn the wheels of the car according to the new_angle    
        ''' The intensity of angle change is preportional to the difference in current angle and desired angle'''
        diff = abs(new_angle - self.car.turnAngle)
        if(new_angle > self.car.turnAngle): # right
            # todo, clarify 0.2 constant
            self.car.changeTurnAngle(self.car.turnAngle + 0.2*diff)
        elif(new_angle < self.car.turnAngle): # left
            self.car.changeTurnAngle(self.car.turnAngle - 0.2*diff)

        # update the start angle for decision nodes
        ''' During a turn, the start_angle is not updated, and is used a reference to measure how much the car has turned'''
        self.start_angle = self.car._angle
        self.start_pos = self.car._position

    def slowProgram(self):
        pass

    #-------------------------------------------------------------------------------------
    def clipAngle(self, angle, clipAmount):
        # angle is given in radians,will clip angle between -clipAmount and clipAmount
        ans = angle
        if angle > clipAmount:
            ans = clipAmount
        elif angle < -clipAmount:
            ans = -clipAmount
        return ans
    
    def _chooseAvoidAngle(self, dist, thresh, buffer):
        # make a list of tuples representing start and end indices of lidar segments
        segments = []
        start = 10
        end = 10
        _sum = 0
        for i in range(10,len(self.car.lidar)-10):
            if(self.car.lidar[i] > dist+thresh):
                end = i+1
                _sum += self.car.lidar[i]
            elif(start != end):
                segments.append( (start,end, _sum) )
                start = end = i+1
                _sum = 0
            else:
                start = end = i+1
        segments.append( (start, end, _sum) )
        # select segment of greatest area
        _max = 0
        index = 0
        for i in range(len(segments)):
            if(segments[i][2] > _max):
                _max = segments[i][2]
                index = i
        # get angle from the middle of the best segment
        size = segments[index][1]-segments[index][0]
        best_index = segments[index][0] + size//2
        angle = math.pi / len(self.car.lidar)
        ans = best_index * angle - math.pi/2
        #draw line just for gui visual
        
        colors = ["#007777","#770077","#777700","#007700","#000077"]
        for seg in range(len(segments)):
            for i in range(segments[seg][0], segments[seg][1]):
                slope = self.car._angle - math.pi/2 + angle*i
                color = "#555500"
                self.screen.create_line(self.car._position[0], self.car._position[1],
                                   self.car._position[0] + math.cos(slope)*self.car.lidar[i],
                                   self.car._position[1] + math.sin(slope)*self.car.lidar[i], fill=colors[seg])

        self.screen.create_line(self.car._position[0], self.car._position[1],
                               self.car._position[0] + math.cos(ans + self.car._angle)*100,
                               self.car._position[1] + math.sin(ans + self.car._angle)*100, fill="#FFFFFF")
        
        return ans

                
    def avoidCollision(self):
        # todo, set to lowest motor speed
        motor_speed = 30
        self.car.changeMotorSpeed(motor_speed)
        buffer = math.pi/15
        new_angle = self._chooseAvoidAngle(self.obsDist, 15, buffer)
        #print("avoid angle: "+str(new_angle))
        #new_angle = self._moveAwayFromWall(10, new_angle)

        if(new_angle > self.car.turnAngle): # right
            # todo
            self.car.changeTurnAngle(self.car.turnAngle + 0.2)#weight*diff)
            #self.turningProgram()
        elif(new_angle < self.car.turnAngle): # left
            self.car.changeTurnAngle(self.car.turnAngle - 0.2)#weight*diff)
        # todo, clarify positions
        # dist = dist_between(self.start_pos, self.car._position)
        # print(dist)
        # print(self.start_pos)
        # if(dist > self.obsDist):
            # self.collisionAvoid = False
        self.collisionAvoid = self.detectObstacle(self._getFrontDist())

    #-------------------------------------------------------------------------------------

    def turningProgram(self):
        #set car velocity preportional to front dist
        #front_dist = self.car.lidar[len(self.car.lidar)//2]
        motor_speed = 30
        self.car.changeMotorSpeed(motor_speed)

        self.detectObstacle(self._getFrontDist())
        
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
            # todo, clarify 0.1 constant
            self.car.changeTurnAngle(self.car.turnAngle + 0.1)#*abs(max_index - self.car.reading_number//2))
        elif(new_angle < self.car.turnAngle): # left
            self.car.changeTurnAngle(self.car.turnAngle - 0.1)#*abs(max_index - self.car.reading_number//2))

    #-------------------------------------------------------------------------------------

    def safetyProgram(self):
        if(self.car.velocity > 0.1): #not stopped
            self.car.changeMotorSpeed(-30)
        self.car.changeMotorSpeed(0)

    #-------------------------------------------------------------------------------------

    def main_funct(self):
        # check for a changed state from Codriver
        if self.codriver is not None:
            self.state = self.codriver.chooseState()
        
        if(not self.safetyMode):
            if(self.collisionAvoid):
                #print("COLLISION AVOID")
                self.avoidCollision()
            elif(self.state == "auto"):
                #print("auto")
                self.autoProgram()
            elif(self.state == "left" or self.state == "right"):
                #print(self.state)
                self.turningProgram()
            elif(self.state =="slow"):
                self.slowProgram()
        else:
            self.safetyProgram()
