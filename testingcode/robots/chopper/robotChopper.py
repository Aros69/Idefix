#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_C, OUTPUT_B,SpeedRPM, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor
from ev3dev2.led import Leds
import os
import time

class RobotChopper:
    # DATA
    _motors = None
    _leftMotor = None
    _rightMotor = None
    _colorSensor = None
    _ultrasonicSensor = None

    # METHODS
    def __init__(self, leftMotor, rightMotor, colorSensor, ultrasonicSensor):
        self._motors = MoveTank(leftMotor, rightMotor)
        self._leftMotor = LargeMotor(leftMotor)
        self._rightMotor = LargeMotor(rightMotor) 
        self._colorSensor = colorSensor(colorSensor)
        self._ultrasonicSensor = UltrasonicSensor(ultrasonicSensor)
        os.system('setfont ' + 'Lat15-Terminus24x12')

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

    def runUntil10cm(self):
        self._motors.run_forever()
        self._motors.on(10, 10)
        while(self._ultrasonicSensor.distance_centimeters>10.0):
            print()    
        self.stopMotors()
    
    def detectColor(self):
        raise NotImplementedError


def main():
    chopper = RobotChopper(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4)
    chopper._colorSensor.
    
    #print(chopper._ultrasonicSensor.distance_centimeters)
    #time.sleep(5)

if __name__ == '__main__':
    main()
    

