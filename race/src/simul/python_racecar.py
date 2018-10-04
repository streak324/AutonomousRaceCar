# python conceptual racecar code for virtual model
import math

deltaTime = 0.033

class Racecar():

    def __init__(self, xPos, yPos, ang):
        '''These variables are not necessarily know by the actual car, but I need for simulation'''
        self._position = (xPos,yPos) # position of the car on the map
        self._angle = ang # angle of car with respect to the map

        '''These variables would be know to the car ahead of time'''
        self.mass = 10 # mass of car
        self.velocity = 0 # direction car is moving (does not have to be same dir it is facing)
        self.motorSpeed = 0 
        self.turnAngle = 0 # angle wheels are facing, 0 is forwards
        self.fricCoeff = 10 # coefficient of friction between ground and tires
        self.carLength = 20 # length of car determines smallest turn radius
        self.reading_number = 90
        self.lidar = [0]*self.reading_number

    def _magnitude(self, vector):
        return math.sqrt(vector[0]**2 + vector[1]**2)

    def _getAngle(self, vector):
        if(vector[0] != 0):
            return math.atan(vector[1] / vector[0])
        else:
            if(vector[1] > 0):
                return math.pi/2
            else:
                return -math.pi/2

    #called to update the velocity of the car as affected by friction
    def _update_friction(self):
        turnRad = math.tan(self.turnAngle + math.pi/2) * self.carLength
        if(abs(self.velocity**2/turnRad) > 9.81*self.fricCoeff):
            self.velocity -= abs(deltaTime * math.sin(self.turnAngle)*9.81*self.fricCoeff)
            #print(self.velocity)

    # called to update the velocity of the car with respect to motor speed
    def _update_motors(self):
        
        turnRad = math.tan(self.turnAngle + math.pi/2) * self.carLength
        '''update velocity based on motor speed'''
        deltaVel = self.motorSpeed - self.velocity
        if(abs(deltaVel) > 9.81*self.fricCoeff*deltaTime):
            if (deltaVel < 0):
                deltaVel = -9.81*self.fricCoeff*deltaTime
            else:
                deltaVel = 9.81*self.fricCoeff*deltaTime
        self.velocity += deltaVel
        
        if((self.turnAngle > 0.01 or self.turnAngle < -0.01) and abs(self.velocity**2/turnRad) <= 9.81*self.fricCoeff):
            '''speed up or slow down car based on motorSpeed'''
            velocity = -self.velocity
            if(self.turnAngle < 0):
                velocity = self.velocity
            radDist = velocity * deltaTime * 2*math.pi / turnRad
            #print("radDist: ", radDist)
            turnCenterX = self._position[0] - math.cos(self._angle + math.pi/2) * turnRad
            turnCenterY = self._position[1] - math.sin(self._angle + math.pi/2) * turnRad
            if(turnRad > 0):
                newPosX = turnCenterX + turnRad * math.cos(self._angle + math.pi/2 - radDist)
                newPosY = turnCenterY + turnRad * math.sin(self._angle + math.pi/2 - radDist)
                self._angle -= radDist
            else:
                newPosX = turnCenterX - turnRad * math.cos(self._angle - math.pi/2 + radDist)
                newPosY = turnCenterY - turnRad * math.sin(self._angle - math.pi/2 + radDist)
                self._angle += radDist
        else:
            newPosX = self._position[0] + self.velocity * deltaTime * 2*math.pi * math.cos(self._angle)
            newPosY = self._position[1] + self.velocity * deltaTime * 2*math.pi * math.sin(self._angle)
        self._position = (newPosX, newPosY)


    #called to update the position, velocity, momentum of the car by one frame
    def _update_pos(self):
        #self._update_friction()
        self._update_motors()
        #self._position = (self._position[0] + self.velocity[0]*deltaTime*10,
        #                  self._position[1] + self.velocity[1]*deltaTime*10)

    def lidar_readings(self, track:"Track"):
        angle = math.pi / self.reading_number
        for i in range(self.reading_number):
            slope = self._angle - math.pi/2 + angle*i
            self.lidar[i] = self.ray_trace(slope, track)

    def ray_trace(self, angle, track:"Track"):
        ray_length = 1
        point = (self._position[0] + math.cos(angle)*ray_length,
                 self._position[1] + math.sin(angle)*ray_length)
        while(not track.check_collision(point)):
            ray_length += 1
            point = (self._position[0] + math.cos(angle)*ray_length,
                     self._position[1] + math.sin(angle)*ray_length)
        return ray_length

    def draw_lidar(self, screen:"tkinter.Canvas"):
        angle = math.pi / self.reading_number
        for i in range(self.reading_number):
            slope = self._angle - math.pi/2 + angle*i
            color = "#555555"
            screen.create_line(self._position[0], self._position[1],
                               self._position[0] + math.cos(slope)*self.lidar[i],
                               self._position[1] + math.sin(slope)*self.lidar[i], fill=color)
            

    def changeMotorSpeed(self, val):
        self.motorSpeed = val

    def changeTurnAngle(self, ang):
        self.turnAngle = ang
        if(self.turnAngle > math.pi/4):
            self.turnAngle = math.pi/4
        if(self.turnAngle < -math.pi/4):
            self.turnAngle = -math.pi/4
