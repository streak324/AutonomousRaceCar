# racecar GUI by Julian Andrews

import tkinter
import math
import python_racecar
import racecar_map
import racecar_ai
import sys

class RaceCarView:

    def __init__(self):
        self.refresh_rate = 10
        self.sim_running = False
        
        self.car = python_racecar.Racecar(75,300, math.pi/2)
        self.track = racecar_map.Track()
        
        
        self.window = tkinter.Tk()
        self.screen = tkinter.Canvas(master = self.window,
                                     width = 800, height = 600,
                                     background = '#000000')
        if "noco" in sys.argv:
            codriver = False
        else:
            codriver = True
        
        if "dumb" in sys.argv:
            dumb = True
        else:
            dumb = False

        self.ai = racecar_ai.RacecarAI(self.car, self.screen, False, False)
        
        self.screen.grid(row = 0, column = 0, sticky = tkinter.NSEW)

        self.window.bind('<Left>', self.left_button_down)
        self.window.bind('<Right>', self.right_button_down)
        self.window.bind('<Up>', self.up_button_down)
        self.window.bind('<Down>', self.down_button_down)
        self.window.bind('<space>', self.pause)
        self.window.bind('n', self.next_frame)

    def _update(self):
        self.screen.delete(tkinter.ALL)
        self.car._update_pos() # update position of car based on physics
        self.car.lidar_readings(self.track)
        self.car.draw_lidar(self.screen)
        self.draw_track()
        self.draw_car()
        self.draw_nodes()
        self.check_crash()
        #AI program
        self.ai.main_funct()
        if(self.sim_running):
            self.screen.after(self.refresh_rate, self._update)

    def start(self):
        self._update()
        self.window.mainloop()

    def pause(self, event:tkinter.Event):
        if self.sim_running:
            self.sim_running = False
        else:
            self.sim_running = True
            self._update()

    def next_frame(self, event:tkinter.Event):
        if self.sim_running:
            return
        self.screen.delete(tkinter.ALL)
        self.car._update_pos() # update position of car based on physics
        self.car.lidar_readings(self.track)
        self.car.draw_lidar(self.screen)
        self.draw_track()
        self.draw_car()
        self.draw_nodes()
        self.check_crash()
        #AI program
        self.ai.main_funct()

    def left_button_down(self, event:tkinter.Event):
        self.car.turnAngle -= 0.1

    def right_button_down(self, event:tkinter.Event):
        self.car.turnAngle += 0.1

    def down_button_down(self, event:tkinter.Event):
        self.car.changeMotorSpeed(-20)
        
    def up_button_down(self, event:tkinter.Event):
        self.car.changeMotorSpeed(20)

    def check_crash(self):
        if(self.track.check_collision(self.car._position)):
            self.sim_running = False
            print("FAILURE")

    def draw_track(self):
        self.track.draw(self.screen)

    def draw_nodes(self):
        if self.ai.codriver is not None:
            for node in self.ai.codriver.nodes:
                self.screen.create_line(node[0], node[1], node[0]+1, node[1]+1, fill="#FFFFFF")

    def draw_car(self):
        centerX = self.car._position[0]
        centerY = self.car._position[1]
        length = self.car.carLength
        axil_length = 10
        angle = math.pi/6
        # Box for car frame
        upperLeftX = centerX + length*math.cos(self.car._angle - angle)
        upperLeftY = centerY + length*math.sin(self.car._angle - angle)
        upperRightX = centerX + length*math.cos(self.car._angle + angle)
        upperRightY = centerY + length*math.sin(self.car._angle + angle)
        lowerLeftX = centerX + length*math.cos(self.car._angle - 5*angle)
        lowerLeftY = centerY + length*math.sin(self.car._angle - 5*angle)
        lowerRightX = centerX + length*math.cos(self.car._angle + 5*angle)
        lowerRightY = centerY + length*math.sin(self.car._angle + 5*angle)
        self.screen.create_line(upperLeftX,upperLeftY,upperRightX,upperRightY, fill = "#FFFFFF")
        self.screen.create_line(upperRightX,upperRightY,lowerRightX,lowerRightY, fill = "#FFFFFF")
        self.screen.create_line(lowerRightX,lowerRightY,lowerLeftX,lowerLeftY, fill = "#FFFFFF")
        self.screen.create_line(lowerLeftX,lowerLeftY,upperLeftX,upperLeftY, fill = "#FFFFFF")
        # Front Axil
        leftX = upperLeftX + axil_length*math.cos(self.car._angle -
                                                  math.pi/2 + self.car.turnAngle)
        leftY = upperLeftY + axil_length*math.sin(self.car._angle -
                                                  math.pi/2 + self.car.turnAngle)
        rightX = upperRightX + axil_length*math.cos(self.car._angle +
                                                    math.pi/2 + self.car.turnAngle)
        rightY = upperRightY + axil_length*math.sin(self.car._angle +
                                                    math.pi/2 + self.car.turnAngle)
        self.screen.create_line(upperLeftX, upperLeftY, leftX, leftY, fill = "#0066FF")
        self.screen.create_line(upperRightX, upperRightY, rightX, rightY, fill = "#0066FF")
        # dotted line
        
        '''update angle based on motorSpeed and turnAngle'''
        turnRad = math.tan(self.car.turnAngle + math.pi/2) * self.car.carLength
        '''speed up or slow down car based on motorSpeed'''
        '''
        #radDist = self.car.velocity * 0.033 * 2*math.pi / turnRad
        turnCenterX = self.car._position[0] - math.cos(self.car._angle + math.pi/2) * turnRad
        turnCenterY = self.car._position[1] - math.sin(self.car._angle + math.pi/2) * turnRad
        #self.screen.create_line(self.car._position[0]+250, self.car._position[1]+250, turnCenterX, turnCenterY, fill = "#00FF00")
        
        for i in range(0,89):
            color = "#00FF00"
            if(turnRad > 0):
                newPosX = turnCenterX + turnRad * math.cos(self.car._angle + math.pi/2 - 0.05*i)
                newPosY = turnCenterY + turnRad * math.sin(self.car._angle + math.pi/2 - 0.05*i)
            else:
                newPosX = turnCenterX - turnRad * math.cos(self.car._angle - math.pi/2 + 0.05*i)
                newPosY = turnCenterY - turnRad * math.sin(self.car._angle - math.pi/2 + 0.05*i)
            self.screen.create_line(newPosX, newPosY, newPosX+1, newPosY+1, fill = color)
        '''
        
        
if __name__ == '__main__':
    display = RaceCarView()
    display.start()
