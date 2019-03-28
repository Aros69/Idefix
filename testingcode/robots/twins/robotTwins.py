#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_C, OUTPUT_B,SpeedRPM, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor
from ev3dev2.led import Leds
import os
import time
from threading import Timer, Thread, Event
from time import sleep


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
        self._colorSensor = ColorSensor(colorSensor)
        self._ultrasonicSensor = UltrasonicSensor(ultrasonicSensor)
        os.system('setfont ' + 'Lat15-Terminus24x12')
        #self._leftMotor.stop_action = "brake"
        #self._rightMotor.stop_action = "brake"

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

    def turnLeftWhenObstacle(self):
        self._motors.run_forever()
        self._motors.on(10,10)
        while(self._ultrasonicSensor.distance_centimeters>10):
            print()
        self.stopMotors()
        self.rightMotorPositiveRotation(10,1)
        self.stopMotors()
    
    def detectColor(self):
        raise NotImplementedError

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
        self._motors.on(-5,-5)

def main():
    chopper = RobotChopper(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4)
    
    # thread must be run at first to start checking the ultra sonique sensor distance
    t2 = Thread(target=chopper.ultrasonicDetect, args=[])
    t2.start()
    
    # thread which runs a common method
    t1 = Thread(target=chopper.runForever, args=[100,100])
    t1.start()

if __name__ == '__main__':
    main()