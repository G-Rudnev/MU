import pygame
import threading
from threading import Lock, Thread
import time
import math



class XboxController(object):
    def __init__(self, type):


        self.type = type

        self.a_pressed = False

        self.axis1X = 0.0
        self.axis1Y = 0.0

        self.axis2X = 0.0
        self.axis2Y = 0.0        

        self.a_pressed = 0
        self.b_pressed = 0
        self.x_pressed = 0
        self.y_pressed = 0

        self.a_released = 0
        self.x_released = 0
        self.b_released = 0
        self.y_released = 0
        self.left_bump_released = 0
        self.right_bump_released = 0
        self.r_down_bump_released = 0
        self.l_down_bump_released = 0 

        self.left_bump_pressed = 0
        self.right_bump_pressed = 0

        self.r_down_bump_pressed = 0
        self.l_down_bump_pressed = 0        

        self.up_hat_released = 0
        self.up_hat_pressed = 0

        pygame.init()
        self.clock = pygame.time.Clock()
        
        self.joysticks = []
        self.joy_connected = False
        self.lastTime = 0
        self.lastActive = 0   


        if self.type == 0:
            self.A_BUTTON = 0
            self.X_BUTTON = 2
            self.Y_BUTTON = 3
            self.B_BUTTON = 1
            self.LEFT_BUMPER = 5
            self.RIGHT_BUMPER = 4
            self.LEFT_SHIFT = 4
            self.RIGHT_SHIFT = 5
            self.HAT_BUTTON = 0
            self.LEFT_AXIS_X = 0
            self.LEFT_AXIS_Y = 1
            self.RIGHT_AXIS_X = 2
            self.RIGHT_AXIS_Y = 3
            
        if self.type == 1:
            self.A_BUTTON = 0
            self.X_BUTTON = 3
            self.Y_BUTTON = 4
            self.B_BUTTON = 1
            self.LEFT_BUMPER = 6
            self.RIGHT_BUMPER = 7
            self.LEFT_SHIFT = 5
            self.RIGHT_SHIFT = 4
            self.HAT_BUTTON = 0
            self.LEFT_AXIS_X = 0
            self.LEFT_AXIS_Y = 1
            self.RIGHT_AXIS_X = 2
            self.RIGHT_AXIS_Y = 3

        if self.type == 2:
            self.A_BUTTON = 0
            self.X_BUTTON = 2
            self.Y_BUTTON = 3
            self.B_BUTTON = 1
            self.LEFT_BUMPER = 4
            self.RIGHT_BUMPER = 5
            self.LEFT_SHIFT = 5
            self.RIGHT_SHIFT = 4
            self.HAT_BUTTON = 0
            self.LEFT_AXIS_X = 0
            self.LEFT_AXIS_Y = 1
            self.RIGHT_AXIS_X = 2
            self.RIGHT_AXIS_Y = 3
        
            
        self.connect()


    def connect(self):
        pygame.joystick.init()
        for i in range(0, pygame.joystick.get_count()):
            self.joysticks.append(pygame.joystick.Joystick(i))
        if len(self.joysticks) > 0:
            self.joy_connected = True
            self.joysticks[-1].init()
            print ("Detected joystick "),self.joysticks[-1].get_name(),"'"
        else:
            pygame.joystick.quit()    

                

    def read(self):
        axis1X = self.axis1X
        axis1Y = self.axis1Y
        axis2X = self.axis2X
        axis2Y = self.axis2Y       
        a = self.a_pressed
        x = self.x_pressed
        y = self.y_pressed
        b = self.b_pressed
        rb = self.right_bump_pressed
        lb = self.left_bump_pressed
        up_hat = self.up_hat_pressed 

        return [axis1X, axis1Y, axis2X, axis2Y, a, x, y, b, rb, lb, up_hat], [self.a_released, self.x_released, self.b_released, self.y_released, self.right_bump_released, self.left_bump_released, self.r_down_bump_released, self.l_down_bump_released, self.up_hat_released]



    def poll_joy(self):
        if self.joy_connected:
            self.a_released = 0
            self.x_released = 0
            self.b_released = 0
            self.y_released = 0
            self.up_hat_released = 0
            self.l_down_bump_released = 0
            self.r_down_bump_released = 0
            for event in pygame.event.get():
                # print(event)
                if event.type == 1542:
                    self.joy_connected = False
                    pygame.joystick.quit()
                    print("Joystick disconnected")
                    self.joysticks = []  

                if event.type == pygame.JOYHATMOTION: 
                    if event.hat == self.HAT_BUTTON:
                        if event.value[1] == 1:   
                            self.up_hat_pressed = 1
                        elif self.up_hat_pressed:    
                            self.up_hat_released = 1 
                            self.up_hat_pressed = 0


                if event.type == pygame.JOYBUTTONDOWN:  
                    if event.button == self.A_BUTTON:
                        self.a_pressed = 1
                    if event.button == self.B_BUTTON:
                        self.b_pressed = 1
                    if event.button == self.X_BUTTON:
                        self.x_pressed = 1
                    if event.button == self.Y_BUTTON:
                        self.y_pressed = 1
                    if event.button == self.RIGHT_BUMPER:
                        self.right_bump_pressed = 1
                    if event.button == self.LEFT_BUMPER:
                        self.left_bump_pressed = 1                                         

                if event.type == pygame.JOYBUTTONUP:
                    if event.button == self.A_BUTTON:
                        if self.a_pressed:
                            self.a_released = 1 
                        self.a_pressed = 0
                    if event.button == self.B_BUTTON:
                        if self.b_pressed:
                            self.b_released = 1                    
                        self.b_pressed = 0
                    if event.button == self.X_BUTTON:
                        if self.x_pressed:
                            self.x_released = 1                    
                        self.x_pressed = 0
                    if event.button == self.Y_BUTTON:
                        if self.y_pressed:
                            self.y_released = 1                    
                        self.y_pressed = 0
                    if event.button == self.RIGHT_BUMPER:
                        if self.y_pressed:
                            self.right_bump_released = 1                    
                        self.right_bump_pressed = 0
                    if event.button == self.LEFT_BUMPER:
                        if self.y_pressed:
                            self.left_bump_released = 1                     
                        self.left_bump_pressed = 0                                                                                                     

                if event.type == pygame.JOYAXISMOTION:
                    if event.axis == self.LEFT_AXIS_X:
                        self.axis1X = event.value
                    if event.axis == self.LEFT_AXIS_Y:
                        self.axis1Y = event.value * -1
                    if event.axis == self.RIGHT_AXIS_X:
                        self.axis2X = event.value * -1
                    if event.axis == self.RIGHT_AXIS_Y:
                        self.axis2Y = event.value * -1
                    
                    if event.axis == self.LEFT_SHIFT:
                        if event.value > 0.9:
                            self.l_down_bump_pressed = 1 
                        else:
                            if self.l_down_bump_pressed:
                                self.l_down_bump_released = 1                    
                            self.l_down_bump_pressed = 0                                                                      

                    if event.axis == self.RIGHT_SHIFT:
                        if event.value > 0.9:
                            self.r_down_bump_pressed = 1 
                        else:
                            if self.r_down_bump_pressed:
                                self.r_down_bump_released = 1                    
                            self.r_down_bump_pressed = 0
                    
        else:
            self.connect()
                                  

    def joystickToDiff(self, x, y, minJoystick, maxJoystick, minSpeed, maxSpeed):
        if (x == 0 and y == 0):  
            return (0, 0)

        if ((x > 0 and x < 0.1) or (x < 0 and x > -0.1)) and ((y > 0 and y < 0.1) or (y < 0 and y > -0.1)):
            return (0, 0)    
     
        
        z = math.sqrt(x * x + y * y)

        rad = math.acos(math.fabs(x) / z)

        angle = rad * 180 / math.pi

        tcoeff = -1 + (angle / 90) * 2
        turn = tcoeff * math.fabs(math.fabs(y) - math.fabs(x))
        turn = round(turn * 100, 0) / 100

        mov = max(math.fabs(y), math.fabs(x))

        if (x >= 0 and y >= 0) or (x < 0 and y < 0):
            rawLeft = mov
            rawRight = turn
        else:
            rawRight = mov
            rawLeft = turn

        if y < 0:
            rawLeft = 0 - rawLeft
            rawRight = 0 - rawRight

        rightOut = self.map(rawRight, minJoystick, maxJoystick, minSpeed, maxSpeed)
        leftOut = self.map(rawLeft, minJoystick, maxJoystick, minSpeed, maxSpeed)

        return leftOut, rightOut


    def map(self, x, oMin, oMax, nMin, nMax):

        if oMin == oMax:
            return None

        if nMin == nMax:
            return None

        reverseInput = False
        oldMin = min(oMin, oMax)
        oldMax = max(oMin, oMax)
        if not oldMin == oMin:
            reverseInput = True

        reverseOutput = False
        newMin = min(nMin, nMax)
        newMax = max(nMin, nMax)
        if not newMin == nMin:
            reverseOutput = True

        portion = (x - oldMin) * (newMax - newMin) / (oldMax - oldMin)
        if reverseInput:
            portion = (oldMax - x) * (newMax - newMin) / (oldMax - oldMin)

        result = portion + newMin
        if reverseOutput:
            result = newMax - portion

        result = max(nMin, result)
        result = min(nMax, result)

        return result


if __name__ == '__main__':
    joy = XboxController(2)
    while True:
        joy.poll_joy()
        time.sleep(0.02)