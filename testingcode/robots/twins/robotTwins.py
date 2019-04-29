#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_C, OUTPUT_B,SpeedRPM, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
from ev3dev2.led import Leds
import os
import time
import sys
from threading import Timer, Thread, Event
from time import sleep


class RobotTwin:
    # DATA
    _motors = None
    _leftMotor = None
    _rightMotor = None
    _colorSensor = None
    _ultrasonicSensor = None
    _gyroSensor = None
    _nbRotationsFor360turnWithOneMotor = 7.8           #à modifier avec les tests, probablement <
    _actualColor = 'q'
    _rightColor = 'q'
    _leftColor = 'q'
    _stopDetectColor = False

    # METHODS
    def __init__(self, leftMotor, rightMotor, colorSensor, ultrasonicSensor, gyroSensor):
        self._motors = MoveTank(leftMotor, rightMotor)
        self._leftMotor = LargeMotor(leftMotor)
        self._rightMotor = LargeMotor(rightMotor) 
        self._colorSensor = ColorSensor(colorSensor)
        self._ultrasonicSensor = UltrasonicSensor(ultrasonicSensor)
        self._gyroSensor = GyroSensor(gyroSensor)
        self._gyroSensor.mode = GyroSensor.MODE_GYRO_CAL
        self._gyroSensor.mode = GyroSensor.MODE_GYRO_ANG
        

    def moveForwardOneSquare(self):
        self.bothMotorsRotation(50,50, 1.8)

    def turnLeft(self):
        #self.bothMotorsRotation(50, -40, -0.6)
        self.bothMotorsRotation(50, -40, -90/146) # 146 = angle après une rotation de moteur


    
    def turnRight(self):
        self.bothMotorsRotation(50, -40, 90/146)
        # t1 = Thread(target=self.rightMotorPositiveRotation, args=[rightPuissance,0.52])
        # t1.start()
        # t2 = Thread(target=self.leftMotorNegativeRotation, args=[leftPuissance,0.52])
        # t2.start()

    def turn180(self):
        self.bothMotorsRotation(50, -40, 180/146)
        # t1 = Thread(target=self.leftMotorPositiveRotation, args=[leftPuissance,1.08])
        # t1.start()
        # t3 = Thread(target=self.rightMotorNegativeRotation, args=[rightPuissance,1.08])
        # t3.start()

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
    
    def getAngle(self):
        return self._gyroSensor.angle 

    def detectColor(self):
        raise NotImplementedError

    # Ultra sonique sensor detect distance from object
    def ultrasonicDetect(self):
        while(self._ultrasonicSensor.distance_centimeters > 20):
            print()
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
    twin = RobotTwin(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4, INPUT_3)

    twin.turnLeft()
    twin.turnRight()
    twin.turn180()
    twin.turn180()
    # thread must be run at first to start checking the ultra sonique sensor distance
    # t2 = Thread(target=twin.ultrasonicDetect, args=[])
    # t2.start()
    
    # # thread which runs a common method
    # t1 = Thread(target=twin.leftMotorPositiveRotation, args=[50,1.08])
    # t1.start()

    # t3 = Thread(target=twin.rightMotorNegativeRotation, args=[50,1.08])
    # t3.start()

if __name__ == '__main__':
    main()