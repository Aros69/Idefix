#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_C, OUTPUT_B,SpeedRPM, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor
from ev3dev2.led import Leds
import os
import sys
import time
from threading import Timer, Thread, Event

class RobotTank:
    # DATA
    _motors = None
    _leftMotor = None
    _rightMotor = None
    _colorSensor = None
    _ultrasonicSensor = None
    _nbRotationsFor360turnWithOneMotor = 7.8
    _actualColor = 'q'
    _rightColor = 'q'
    _leftColor = 'q'
    _stopDetectColor = False

    # METHODS
    def __init__(self, leftMotor, rightMotor, colorSensor, ultrasonicSensor):
        self._motors = MoveTank(leftMotor, rightMotor)
        self._leftMotor = LargeMotor(leftMotor)
        self._rightMotor = LargeMotor(rightMotor)
        self._colorSensor = ColorSensor(colorSensor)
        self._ultrasonicSensor = UltrasonicSensor(ultrasonicSensor)

    def turnLeft(self, leftPuissance, rightPuissance, rotation):
        raise NotImplementedError
    
    def turnRight(self, leftPuissance, rightPuissance, rotation):
        raise NotImplementedError

    def turn180(self, leftPuissance, rightPuissance, rotation):
        raise NotImplementedError
    
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
        if(self._colorSensor.red>=130 and self._colorSensor.red<=140):
            self._actualColor = 'r'
        elif (self._colorSensor.red>=10 and self._colorSensor.red<=40):
            self._actualColor = 'n'
        elif (self._colorSensor.red>=190 and self._colorSensor.red<=220):
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
            self.stopMotors()
            self._rightColor = self._actualColor
            # self.runForever(0, 25)
            # while(self._actualColor==self._rightColor):
            #     print()
            # self.stopMotors()
            # self._leftColor = self._actualColor
        else:
            self.runForever(25, 0)
            while(self._actualColor=='q'):
                print()
            self.stopMotors()
            self._leftColor = self._actualColor
            # self.runForever(25, 0)
            # while(self._actualColor==self._leftColor):
            #     print()
            # self.stopMotors()
            # self._rightColor = self._actualColor
        print(self._leftColor, file=sys.stderr)
        print(self._rightColor, file=sys.stderr)

    def runStraigthLine(self):
        threadWallDetection = Thread(target=self.ultrasonicDetect, args=[])
        threadWallDetection.start()
        threadColorDetection = Thread(target=self.nonStopDetectColor, args=[])
        threadColorDetection.start()
        if(self._rightColor=='b'):
            if(self._actualColor!=self._leftColor):
                self.findLine('r')
            self.runForever(50, 50)
            while(self._motors.is_running):
                if(self._actualColor=='b'):
                    self.runForever(20, 50)
                elif self._actualColor=='q':
                    self.runForever(50, 20)
                else:
                    self.runForever(50, 50)
        else:
            if(self._actualColor!=self._rightColor):
                self.findLine('l')
            self.runForever(50, 50)
            while(self._motors.is_running):
                if(self._actualColor=='b'):
                    self.runForever(50, 20)
                elif self._actualColor=='q':
                    self.runForever(20, 50)
                else:
                    self.runForever(50, 50)
    

def main():
    tank = RobotTank(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4)
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
    tank.findLine('l')
    
            


if __name__ == '__main__':
    main()
    

