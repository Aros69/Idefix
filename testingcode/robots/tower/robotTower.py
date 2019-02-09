#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_C, OUTPUT_B,SpeedRPM, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, GyroSensor
from ev3dev2.led import Leds
import time

class RobotTower:
    # DATA
    _motors = None
    _leftMotor = None
    _rightMotor = None

    # METHODS
    def __init__(self, leftMotor, rightMotor):
        self._motors = MoveTank(leftMotor, rightMotor)
        self._leftMotor = LargeMotor(leftMotor)
        self._rightMotor = LargeMotor(rightMotor)

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
        raise NotImplementedError


def main():
    tower = RobotTower(OUTPUT_A, OUTPUT_D)
    tower.bothMotorsRotation(50,50,2)

if __name__ == '__main__':
    main()
    
