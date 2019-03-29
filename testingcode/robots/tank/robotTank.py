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

    def turnLeft(self):
        self.bothMotorsRotation(30, -30, -0.85)
    
    def turnRight(self):
        self.bothMotorsRotation(30, -30, 0.85)

    def turn180(self):
        self.bothMotorsRotation(30, -30, 1.7)

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
    
    #tank.findLineDumb()
    tank.runStraigthLine()
    print("End of the fucking program !", file=sys.stderr)
            


if __name__ == '__main__':
    main()
    

