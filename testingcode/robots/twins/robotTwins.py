#!/usr/bin/env python3

from ev3dev2 import get_current_platform
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_C, OUTPUT_B,SpeedRPM, SpeedPercent, MoveTank
from ev3dev2.sensor import Sensor, INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
from ev3dev2.led import Leds
import os
import time
import sys
import random
from threading import Timer, Thread, Event
from time import sleep


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
    _isWallAhead = True
    _threadSonic = None
    _threadColor = None
    _actualColor = 0
    _rightColor = 0
    _leftColor = 0
    _stopDetectColor = False
    _stopThread = False

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
        self._threadSonic = Thread(target=self.setIsWallAhead, args=[])
        self._threadColor = Thread(target=self.setColor, args=[])
        self._threadSonic.start()
        sleep(1)

    def moveForwardOneSquare(self):
        if(not(self._isWallAhead)):
            self.bothMotorsRotation(50,50, 1.8)
            self.orientationCorrection()

    def turnLeft(self):
        #self.bothMotorsRotation(50, -40, -0.6)
        angleObjectif = (self._baseAngle+90)%360
        self.bothMotorsRotation(50, -40, -90/146) # 146 = facteur de rotation d'après une rotation de moteur
        time.sleep(1)
        self._actualAngle = self._compassSensor.value()
        self._baseAngle = angleObjectif
        self.orientationCorrection()

    def turnRight(self):
        angleObjectif = (self._baseAngle-90)%360
        self.bothMotorsRotation(50, -40, 90/146) # 146 = facteur de rotation d'après une rotation de moteur
        time.sleep(1)
        self._actualAngle = self._compassSensor.value()
        self._baseAngle = angleObjectif
        self.orientationCorrection()

    def orientationCorrection(self):
        sleep(0.2)
        angleDif = self._baseAngle - self._compassSensor.value()
        if(angleDif<-1 or angleDif>1):
            while(self._compassSensor.value()<self._baseAngle-1 or self._compassSensor.value()>self._baseAngle+1):
                if(self._compassSensor.value()>self._baseAngle and self._compassSensor.value()<self._baseAngle+180%360):
                    self.bothMotorsRotation(5, -4, 2/146)
                elif(self._compassSensor.value()<self._baseAngle and self._compassSensor.value()>self._baseAngle-180%360) :
                    self.bothMotorsRotation(5, -4, -2/146)
                else :
                    #print("Je sais pas quoi faire", file=sys.stderr)
                    self.bothMotorsRotation(5, -4, -2/146)
                #print("objectif = ", self._baseAngle, " actuel = ", self._compassSensor.value(), file=sys.stderr)
                sleep(0.5)
                

    def turn180(self):
        angleObjectif = (self._baseAngle-180)%360
        self.bothMotorsRotation(50, -40, 180/146) # 146 = facteur de rotation d'après une rotation de moteur
        time.sleep(1)
        self._actualAngle = self._compassSensor.value()
        self._baseAngle = angleObjectif
        self.orientationCorrection()

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

    def stopMotors(self):
        self._motors.off()
    
    def getAngle(self):
        self._actualAngle = self._compassSensor.value()
        return self._compassSensor.value()

    def color(self):
        if (self._colorSensor.color == self._colorSensor.COLOR_BLACK):
            self._actualColor = self._colorSensor.COLOR_BLACK
            # print("couleur (noire) = ", self._actualColor, file=sys.stderr)
        elif (self._colorSensor.color == self._colorSensor.COLOR_RED):
            self._actualColor = self._colorSensor.COLOR_RED
            # print("couleur (rouge) = ", self._actualColor, file=sys.stderr)
        elif (self._colorSensor.color == self._colorSensor.COLOR_WHITE):
            self._actualColor = self._colorSensor.COLOR_WHITE
            # print("couleur (blanc) = ", self._actualColor, file=sys.stderr)
        else:
            self._actualColor = self._colorSensor.COLOR_BROWN
            # print("couleur (marron) = ", self._actualColor, file=sys.stderr)

    def scanColor(self):
        if (self._actualColor != self._colorSensor.COLOR_BROWN):
            # balayage jusqu'a marron
            while (self._actualColor != self._colorSensor.COLOR_BROWN):
                # Tourne à gauche (normalement)
                self.runForever(-6, 6)
                # self.bothMotorsRotation(12, -12, -0.05)
            self.stopMotors()
            print("couleur = ", self._actualColor, file=sys.stderr)
        while (self._actualColor == self._colorSensor.COLOR_BROWN):
            # Tourne à droite (normalement)
            self.runForever(6, -6)
            # self.bothMotorsRotation(12, -12, 0.05)
        self.stopMotors()
        self._leftColor = self._actualColor
        print("couleur = ", self._actualColor, file=sys.stderr)
        while (self._actualColor == self._leftColor or self._actualColor == self._colorSensor.COLOR_BROWN):
            # Tourne à droite (normalement)
            self.runForever(6, -6)
            # self.bothMotorsRotation(12, -12, 0.05)
        self.stopMotors()
        self._rightColor = self._actualColor
        print("couleur = ", self._actualColor, file=sys.stderr)
        while (self._actualColor != self._leftColor):
            self.runForever(-6, 6)
        self.stopMotors()
        # self.bothMotorsRotation(12, -12, -0.05)

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
        while(not(self._stopThread)):
            self._actualAngle = self._compassSensor.value()

    def setIsWallAhead(self):
        while(not(self._stopThread)):
            self._isWallAhead = self._ultrasonicSensor.distance_centimeters<10
            #print(self._ultrasonicSensor.distance_centimeters, file=sys.stderr)

    def setColor(self):
        while (not (self._stopThread)):
            self.color()

def main():
    twin = RobotTwin(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4, INPUT_3)

    twin.scanColor()

    '''
    i=0
    x=-1
    while(i<100):
        x = random.randrange(5)
        if(x==0 or x==1):
            twin.moveForwardOneSquare()
        elif (x==2):
            twin.turnLeft()
        elif (x==3):
            twin.turnRight()
        else:
            twin.turn180()
        i+=1
        print(i, file=sys.stderr) '''
    twin._stopThread = True


if __name__ == '__main__':
    main()