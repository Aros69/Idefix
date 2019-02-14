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
        self._motors.off
    
    def detectColor(self):
        if(self._colorSensor.red>=130 and self._colorSensor.red<=140):
            return 'r'
        elif (self._colorSensor.red>=10 and self._colorSensor.red<=40):
            return 'n'
        elif (self._colorSensor.red>=190 and self._colorSensor.red<=220):
            return 'b'
        else:
            return 'q'

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


def main():
    tank = RobotTank(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4)
    #tank.bothMotorsRotation(50,-30,1)
    #tank.oneTurnColorDetection(0.1, 'r')
    #time.sleep(5)
    #tank.oneTurnColorDetection(0.1, 'l')

    # thread must be run at first to start checking the ultra sonique sensor distance
    t2 = Thread(target=tank.ultrasonicDetect, args=[])
    t2.start()
    
    # thread which runs a common method
    t1 = Thread(target=tank.runForever, args=[50,50])
    t1.start()

    #tank._motors.run_forever
    #tank._motors.on(-20, -20)
    #temp = 0
    #while(tank._ultrasonicSensor.distance_centimeters>20.0):
    #    temp+=1
    #tank._motors.off
    #print(tank._ultrasonicSensor.distance_centimeters, file=sys.stderr)

    #i=0
    #while(tank.detectColor()=='q' and i<100):
    #    tank.rightMotorNegativeRotation(50, 0.1)
    #    i=i+1
    #print(tank.detectColor(), file=sys.stderr)


if __name__ == '__main__':
    main()
    

