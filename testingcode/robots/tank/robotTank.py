#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_C, OUTPUT_B,SpeedRPM, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
from ev3dev2.led import Leds
import os
import sys
import time
from threading import Timer, Thread, Event

# sys.path.insert(0, "/your/path/to/idefix") # if you want to execute main in this file
sys.path.insert(0, "/home/baoanh/Travail/Master1/idefix/")
from testingcode.exploration.labyrinthe import Labyrinthe

class RobotTank:
    # DATA
    _motors = None
    _leftMotor = None
    _rightMotor = None
    _colorSensor = None
    _ultrasonicSensor = None
    _gyroSensor = None
    _nbRotationsFor360turnWithOneMotor = 7.8
    _actualColor = 'q'
    _rightColor = 'q'
    _leftColor = 'q'
    _angleForward = 0
    _stopDetectColor = False
    _labyrinthe = None
    _position = None # Tuple (i,j)

    # METHODS
    def __init__(self, leftMotor, rightMotor, colorSensor, ultrasonicSensor, gyroSensor, position, labyrinthe):
        self._motors = MoveTank(leftMotor, rightMotor)
        self._leftMotor = LargeMotor(leftMotor)
        self._rightMotor = LargeMotor(rightMotor)
        self._colorSensor = ColorSensor(colorSensor)
        self._ultrasonicSensor = UltrasonicSensor(ultrasonicSensor)
        self._gyroSensor = GyroSensor(gyroSensor)
        self._gyroSensor.mode = GyroSensor.MODE_GYRO_ANG
        self._gyroSensor.reset
        self._angleForward = self._gyroSensor.angle
        self._position = position
        # self._labyrinthe = Labyrinthe(0, 0, 8,8, position)
        self._labyrinthe = labyrinthe

        self._labyrinthe.init2DGraph()

    def turnLeft(self):
        self.bothMotorsRotation(30, -30, -0.85)
        self._angleForward-=90
    
    def turnRight(self):
        self.bothMotorsRotation(30, -30, 0.85)
        self._angleForward+=90

    def turn180(self):
        self.bothMotorsRotation(30, -30, 1.7)
        self._angleForward+=180

    def moveForwardOneSquare(self):
        self.bothMotorsRotation(50,48, 2.7) #Puisance moteur droit légerement moins puissant pour compenser un soucis
    
    def bothMotorsRotation(self, leftPuissance, rightPuissance, rotation):
        self._motors.on_for_rotations(SpeedPercent(leftPuissance), SpeedPercent(rightPuissance), rotation)
        time.sleep(1)
    
    def rightMotorPositiveRotation(self, puissance, rotation):
        self._rightMotor.on_for_rotations(SpeedPercent(puissance), rotation)
        time.sleep(1)
    
    def rightMotorNegativeRotation(self, puissance, rotation):
        self._rightMotor.on_for_rotations(SpeedPercent(-puissance), rotation)
        time.sleep(1)
    
    def leftMotorPositiveRotation(self, puissance, rotation):
        self._leftMotor.on_for_rotations(SpeedPercent(puissance), rotation)
        time.sleep(1)
    
    def leftMotorNegativeRotation(self, puissance, rotation):
        self._leftMotor.on_for_rotations(SpeedPercent(-puissance), rotation)
        time.sleep(1)
    
    def stopLeftMotor(self):
        self._leftMotor.off
    
    def stopRightMotor(self):
        self._rightMotor.off

    def stopMotors(self):
        self._stopDetectColor = True
        self._motors.off
    
    def detectColor(self):
        if(self._colorSensor.red>=125 and self._colorSensor.red<=160):
            self._actualColor = 'r'
        elif (self._colorSensor.red>=10 and self._colorSensor.red<=60):
            self._actualColor = 'n'
        elif (self._colorSensor.red>=190 and self._colorSensor.red<=230):
            self._actualColor = 'b'
        else:
            self._actualColor = 'q'

    def nonStopDetectColor(self):
        self._stopDetectColor = False
        while(not(self._stopDetectColor)):
            self.detectColor()

    def oneTurnColorDetection(self, rotationStep, side):
        i = 0
        if (side=='r'):
            while (i < self._nbRotationsFor360turnWithOneMotor):
                self.rightMotorNegativeRotation(50, rotationStep)
                i+=rotationStep
        else: 
            while (i < self._nbRotationsFor360turnWithOneMotor):
                self.leftMotorNegativeRotation(50, rotationStep)
                i+=rotationStep
        return self.detectColor()
    
     # Ultra sonique sensor detect distance from object
    def ultrasonicDetect(self):
        while(self._ultrasonicSensor.distance_centimeters > 20):
            print()
        self.runForeverAbsorbEnergy()
        self.stopMotors()

    # Motors will run until it dies or receive command to stop
    def runForever(self, leftPower, rightPower):
        self._motors.run_forever()
        self._motors.on(leftPower,rightPower)

    # Allow the robot to do a little backward avoiding to hit the wall
    def runForeverAbsorbEnergy(self):
        self._motors.run_forever()
        self._motors.on(-8,-8)
    
    def findLine(self, side):
        threadFindLine1 = Thread(target=self.nonStopDetectColor, args=[])
        threadFindLine1.start()
        if (side=='l'):
            self.runForever(0, 25)
            while(self._actualColor=='q'):
                print()
            self._rightColor = self._actualColor
            time.sleep(2)
            self.runForever(0, -25)
            while(self._actualColor=='q'):
                print()
            self._leftColor = self._actualColor
            time.sleep(2)
            self.stopMotors()
        else:
            # do same for right turn
            print()
        print(self._leftColor, file=sys.stderr)
        print(self._rightColor, file=sys.stderr)

    def findLineDumb(self):
        #threadColorDetection = Thread(target=self.nonStopDetectColor, args=[])
        #threadColorDetection.start()
        self.detectColor()
        self._rightColor = self._actualColor
        self.bothMotorsRotation(-50, 30, 0.14) # 0,14 is a special value find by testing 
        time.sleep(2)
        self.detectColor()
        self._leftColor = self._actualColor 
        self.bothMotorsRotation(-50, 30, -0.14) # 0,14 is a special value find by testing 
        print("left color : " + self._leftColor + ", right color :"+self._rightColor, file=sys.stderr)


    def runStraigthLine(self):
        self.findLineDumb()
        threadWallDetection = Thread(target=self.ultrasonicDetect, args=[])
        threadWallDetection.start()
        threadColorDetection = Thread(target=self.nonStopDetectColor, args=[])
        threadColorDetection.start()
        powerLeftMotor = 20
        powerRightMotor= 20
        if(self._leftColor!=self._rightColor and self._leftColor!='q' and self._rightColor!='q'):
            print("Let's go !", file=sys.stderr)
            wasBlack=False
            self.runForever(powerLeftMotor, powerRightMotor)
            while(self._motors.is_running):
                self.runForever(powerLeftMotor, powerRightMotor)
                if(self._actualColor==self._leftColor):
                    wasBlack=True
                    print(self._actualColor, " >>>>>", file=sys.stderr)
                    powerRightMotor= 13
                elif(self._actualColor=='q'):
                    if(wasBlack):
                        print(self._actualColor, " >>>>>", file=sys.stderr)
                        powerRightMotor= 13
                    else:
                        print(self._actualColor, " <<<<<", file=sys.stderr)
                        powerLeftMotor= 13
                else:
                    wasBlack=False
                    print(self._actualColor, " =====", file=sys.stderr)
                    powerLeftMotor=20
                    powerRightMotor=20
                    
            self.stopMotors()
        else:
            print("Fuck.", file=sys.stderr)
        print("The end.", file=sys.stderr)
    

    '''
    robot will try to follow the path given. if block then update labyrinth

    :param: path a array of node
    :return: true if final node achieved else false
    '''
    def robot_follow(self, path):
        # Loop throught path Attention, path contain actual position of robot
        # Get direction of each next node by using self._labyrinthe.direction()
        # Move robot
        # if block, update position of robot in self._labyrinthe
        # if block, remove edge using self._labyrinthe.graph.remove_edge()
        # if block, return False

        # if not block, update position of robot
        # if not block, return True

        return True

    '''
    robot will check if the edge around is blocked
    :return: list of not accesible edge, else empty list
    '''
    def robot_scan(self):
        # do a 360 to check if edge exist (no wall)
        # if not exist add to edges list, format [( (0,0), (0,1) ), ( (0,0), (1,1) ), ...]
        return []

    #TODO à vérifié avec l'algo bidon
    '''
    :return: True if labyrinthe corrected else other robot have to help
    '''
    def correct_labyrinth(self):
        
        # calculate all shortest path.
        to_visit = self._labyrinthe.not_visited_node()
        path_block = False

        while len(to_visit) > 0 and not path_block:
            path = self._labyrinthe.nearest_node(to_visit)
            
            # if there are a path
            if len(path) > 0:
                sucess = self.robot_follow(path)

                if (sucess):
                    edges = self.robot_scan()
                    self._labyrinthe.graph.remove_edges_from(edges)
                    to_visit.remove((path[-1]))
            else:
                path_block = True
        
        #to_visit is now a list of not achieved path that other robot need to check
        return not path_block, to_visit


        # transforme into robot direction mouvement
        # if chemin don't exist.
        # update graph then loop
        # graph.remove_edge
        
        # if destinations.
        # scan, update graph
        # remove node from not visited, then check for next shortest path.

    def get_position(self):
        return self._position

    def get_labyrinthe(self):
        return self._labyrinthe


def main():
    pass
    tank = RobotTank(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4, INPUT_3, (0,0)) #to decoment
    #tank.bothMotorsRotation(50,-30,1)

    # thread must be run at first to start checking the ultra sonique sensor distance
    #t2 = Thread(target=tank.ultrasonicDetect, args=[])
    #t2.start()
    
    # thread which runs a common method
    #t1 = Thread(target=tank.runForever, args=[50,50])
    #t1.start()

    
    
    #temp = tank._actualColor
    #oldTemp = temp
    #while(True):
    #    oldTemp=temp
    #    temp=tank._actualColor
    #    if(temp!=oldTemp):
    #        print(temp, file=sys.stderr)
    #tank.findLine('l')
    
    #tank.findLineDumb()
    #tank.runStraigthLine()
    # while(True): # to decomment
    #     print(tank._gyroSensor.angle, file=sys.stderr)
    #     time.sleep(5)
    # print("End of the fucking program !", file=sys.stderr)
            


if __name__ == '__main__':
    main()
    

