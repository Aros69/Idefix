#!/usr/bin/env python3

from ev3dev2 import get_current_platform

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_C, OUTPUT_B,SpeedRPM, SpeedPercent, MoveTank
from ev3dev2.sensor import Sensor, INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
from ev3dev2.led import Leds
import os
import time
import sys
from threading import Timer, Thread, Event
from time import sleep

#sys.path.append('../../../include/nxt-python-2.2.2/nxt/sensor')
#from hitechnic import Compass


class RobotTwin:
    # DATA
    _motors = None
    _leftMotor = None
    _rightMotor = None
    _colorSensor = None
    _ultrasonicSensor = None
    _compassSensor = None
    _baseAngle = 0
    _actualAngle = 0
    _threadCompass = None
    #_gyroSensor = None
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
        self._compassSensor = Sensor(INPUT_2)
        sleep(1)
        self._baseAngle = self._compassSensor.value()
        self._actualAngle = self._baseAngle
        self._threadCompass = Thread(target=self.setActualAngle, args=[])
        # self._gyroSensor = GyroSensor(gyroSensor)
        # self._gyroSensor.mode = GyroSensor.MODE_GYRO_CAL
        # self._gyroSensor.mode = GyroSensor.MODE_GYRO_ANG
        
    def moveForwardOneSquare(self):
        self.bothMotorsRotation(50,50, 1.8)

    def turnLeft(self):
        #self.bothMotorsRotation(50, -40, -0.6)
        self.bothMotorsRotation(50, -40, -90/146) # 146 = angle après une rotation de moteur

    def turnRight(self):
        angleObjectif = (self._baseAngle-90)%360
        while(self._actualAngle<angleObjectif-10 or self._actualAngle>angleObjectif+10):
            self.runForever(50,-40)
            #print("base : ", self._baseAngle, " va jusqu'a : ", angleObjectif, " evolution : ", self.getAngle(), file=sys.stderr)
            self._actualAngle = self._compassSensor.value()
        self._baseAngle = angleObjectif
        self.stopMotors()

    def orientationCorrection(self):
        sleep(0.2)
        angleDif = self._baseAngle - self._compassSensor.value()
        if(angleDif<-1 or angleDif>1):
            angleDif = self._baseAngle - self._compassSensor.value()
            #while(self._compassSensor.value()!=angleDif):
            while(self._compassSensor.value()<self._baseAngle-1 or self._compassSensor.value()>self._baseAngle+1):
                self.bothMotorsRotation(5, -4, 0.01)
                sleep(0.1)



            

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
        #return self._gyroSensor.angle 
        return self._actualAngle

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

    def setActualAngle(self):
        while(True):
            self._actualAngle = self._compassSensor.value()

def main():
    twin = RobotTwin(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4, INPUT_3)

    #print(x, file=sys.stderr)
    #print(x.value(), file=sys.stderr)

    #twin.turnRight()
    twin.moveForwardOneSquare()
    twin.orientationCorrection()

    #print(x.mode, file=sys.stderr)
    #print(x.value(), file=sys.stderr)

    #twin.turnRight()
    #print(x.value(), file=sys.stderr)
    #x.mode = b"BEGIN-CAL"
    
    #print(ev3dev_sysinfo, file=sys.stderr)

    #twin.turnLeft()
    #twin.turnRight()
    #twin.turn180()
    #twin.turn180()

    # thread must be run at first to start checking the ultra sonique sensor distance
    # t2 = Thread(target=twin.ultrasonicDetect, args=[])
    # t2.start()
    
    # # thread which runs a common method
    # t1 = Thread(target=twin.leftMotorPositiveRotation, args=[50,1.08])
    # t1.start()

    # t3 = Thread(target=twin.rightMotorNegativeRotation, args=[50,1.08])
    # t3.start()

    #x.command = 'BEGIN-CAL'
    #twin.turn1080()
    #x.command = 'END-CAL'

if __name__ == '__main__':
    main()