#!/usr/bin/env python3

from ev3dev2 import get_current_platform
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_C, OUTPUT_B,SpeedRPM, SpeedPercent, MoveTank
from ev3dev2.sensor import Sensor, INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
from ev3dev2.led import Leds
import os, sys, time, random
from threading import Timer, Thread, Event
from time import sleep

sys.path.append('../../')
from chronometre import Chrono

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

    ''' 0 = Robot orienté face au mur de la TD 6 (couleur droite = noire et couleur gauche = blanc 
    1 = Robot orienté face au couloir (couleur droite = noire et couleur gauche = rouge 
    2 = Robot orienté face au bureau du prof (couleur droite = blanc et couleur gauche = noire 
    3 = Robot orienté face au fenetre (couleur droite = rouge et couleur gauche = noire '''
    _cardinalPoint = -1

    _stopDetectColor = False
    _stopThread = False

    # METHODS
    def __init__(self, leftMotor, rightMotor, colorSensor, ultrasonicSensor, compassSensor):
        self._motors = MoveTank(leftMotor, rightMotor)
        self._leftMotor = LargeMotor(leftMotor)
        self._rightMotor = LargeMotor(rightMotor) 
        self._colorSensor = ColorSensor(colorSensor)
        self._ultrasonicSensor = UltrasonicSensor(ultrasonicSensor)
        self._compassSensor = Sensor(compassSensor)
        sleep(1)
        self._baseAngle = self._compassSensor.value()
        self._actualAngle = self._baseAngle
        self._threadSonic = Thread(target=self.setIsWallAhead, args=[])
        self._threadColor = Thread(target=self.setColor, args=[])
        self._threadSonic.start()
        self._threadColor.start()
        sleep(1)
        self.scanColor()
        self.setCardinalPoint()

    ''' 0 = Robot orienté face au mur de la TD 6 (couleur droite = noire et couleur gauche = blanc 
    1 = Robot orienté face au couloir (couleur droite = noire et couleur gauche = rouge 
    2 = Robot orienté face au bureau du prof (couleur droite = blanc et couleur gauche = noire 
    3 = Robot orienté face au fenetre (couleur droite = rouge et couleur gauche = noire '''
    def setCardinalPoint(self):
        if(self._leftColor==self._colorSensor.COLOR_WHITE):
            self._cardinalPoint = 0
        elif(self._leftColor==self._colorSensor.COLOR_RED):
            self._cardinalPoint = 1
        elif(self._rightColor==self._colorSensor.COLOR_WHITE):
            self._cardinalPoint = 2
        else: 
            self._cardinalPoint = 3

    ''' 0 = Robot orienté face au mur de la TD 6 (couleur droite = noire et couleur gauche = blanc 
    1 = Robot orienté face au couloir (couleur droite = noire et couleur gauche = rouge 
    2 = Robot orienté face au bureau du prof (couleur droite = blanc et couleur gauche = noire 
    3 = Robot orienté face au fenetre (couleur droite = rouge et couleur gauche = noire '''
    def setColorWithCardinalPoint(self):
        if(self._cardinalPoint == 0):
            self._leftColor = self._colorSensor.COLOR_WHITE
            self._rightColor = self._colorSensor.COLOR_BLACK
        elif(self._cardinalPoint == 1):
            self._leftColor = self._colorSensor.COLOR_RED
            self._rightColor = self._colorSensor.COLOR_BLACK
        elif(self._cardinalPoint == 2):
            self._leftColor = self._colorSensor.COLOR_BLACK
            self._rightColor = self._colorSensor.COLOR_WHITE
        else: 
            self._leftColor = self._colorSensor.COLOR_BLACK
            self._rightColor = self._colorSensor.COLOR_RED

    def moveForwardOneSquare(self):
        if(not(self._isWallAhead)):
            self.bothMotorsRotation(25,25, 1.8)
            self.orientationCorrection()

    def moveForwardOneSquare2(self):
        if(not(self._isWallAhead)):
            # run forever with timer and modification of power
            leftPower = 25
            rightPower = 25
            inCorrection = False
            compenseMarron = False
            c = Chrono()
            c.start()
            while(c.getTime() < 2.4) :# timer < nbSeconde pour atteindre une case
                if(not(inCorrection)):
                    if(self._actualColor==self._rightColor):
                        leftPower = 22
                        rightPower = 27
                        inCorrection = True
                        print("Compense vers gauche", file=sys.stderr)
                    elif(self._actualColor==self._colorSensor.COLOR_BROWN):
                        leftPower = 27
                        rightPower = 22
                        inCorrection = True
                        compenseMarron = True
                        print("Compense vers droite", file=sys.stderr)
                else:
                    if(self._actualColor==self._leftColor):
                        rightPower = 25
                        leftPower = 25
                        inCorrection = False
                        compenseMarron = False
                        print("Pas de compensation", file=sys.stderr)
                    elif(compenseMarron and self._actualColor==self._rightColor):
                        leftPower = 22
                        rightPower = 27
                        inCorrection = True
                        compenseMarron = False
                        print("Erreur Compensation, tentative de correction", file=sys.stderr)
                self.runForever(leftPower, rightPower)
            self.stopMotors()
            sleep(1)
            self.orientationCorrection()
        sleep(1)

    def turnLeft(self):
        #self.bothMotorsRotation(50, -40, -0.6)
        angleObjectif = (self._baseAngle+90)%360
        self._cardinalPoint=(self._cardinalPoint-1)%4
        self.setColorWithCardinalPoint()
        self.bothMotorsRotation(25, -20, -85/146) # 146 = facteur de rotation d'après une rotation de moteur
        time.sleep(1)
        self._actualAngle = self._compassSensor.value()
        self._baseAngle = angleObjectif
        self.orientationCorrection()
        #print("Orientation = ", self._cardinalPoint, file=sys.stderr)

    def turnRight(self):
        angleObjectif = (self._baseAngle-90)%360
        self._cardinalPoint=(self._cardinalPoint+1)%4
        self.setColorWithCardinalPoint()
        self.bothMotorsRotation(25, -20, 85/146) # 146 = facteur de rotation d'après une rotation de moteur
        time.sleep(1)
        self._actualAngle = self._compassSensor.value()
        self._baseAngle = angleObjectif
        self.orientationCorrection()
        #print("Orientation = ", self._cardinalPoint, file=sys.stderr)


    def turn180(self):
        angleObjectif = (self._baseAngle-180)%360
        self._cardinalPoint=(self._cardinalPoint+2)%4
        self.setColorWithCardinalPoint()
        self.bothMotorsRotation(25, -20, 170/146) # 146 = facteur de rotation d'après une rotation de moteur
        time.sleep(1)
        self._actualAngle = self._compassSensor.value()
        self._baseAngle = angleObjectif
        self.orientationCorrection()
        print("Orientation = ", self._cardinalPoint, file=sys.stderr)

    def orientationCorrection(self):
        sleep(0.2)
        angleDif = self._baseAngle - self._compassSensor.value()
        if(angleDif<-1 or angleDif>1 or self._actualColor != self._leftColor):
            # On enregistre la dernière action pour la continuer si on a un doute
            lastAction = -1
            i = 2
            while(lastAction==-1 and i<60):
                if(self._compassSensor.value()==(self._baseAngle+i)%360): 
                    lastAction = 1
                i+=1
            if(lastAction==-1):
                i = 2
                while(lastAction==-1 and i<60):
                    if(self._compassSensor.value()==(self._baseAngle-i)%360): 
                        lastAction = 2
                    i+=1
            while self._actualColor != self._leftColor:
                if lastAction==1 :
                    self.bothMotorsRotation(5, -4, 0.05)
                elif lastAction==2:
                    self.bothMotorsRotation(5, -4, -0.05)
                else:    
                    print("Je sais pas quoi faire", file=sys.stderr)
                    print("Angle actuel = ", self._compassSensor.value(), file=sys.stderr)
                    print("Angle de base = ", self._baseAngle, file=sys.stderr)
                sleep(0.5)
            '''if lastAction==1 :
                self.bothMotorsRotation(5, -4, 2/146)
            elif lastAction==2:
                self.bothMotorsRotation(5, -4, -2/146)'''
        #print("Couleur actuel = ", self._actualColor, "Couleur gauche = ", self._leftColor, file=sys.stderr)
        #print("Angle actuel = ", self._compassSensor.value(), "Angle attendu = ", self._baseAngle, file=sys.stderr)

    def bothMotorsRotation(self, leftPuissance, rightPuissance, rotation):
        self._motors.on_for_rotations(SpeedPercent(leftPuissance), SpeedPercent(rightPuissance), rotation)
        time.sleep(1)

    def stopMotors(self):
        self._motors.off()
    
    def getAngle(self):
        self._actualAngle = self._compassSensor.value()
        return self._compassSensor.value()

    def color(self):
        if (self._colorSensor.color == self._colorSensor.COLOR_BLACK):
            self._actualColor = self._colorSensor.COLOR_BLACK
            #print("couleur (noire) = ", self._actualColor, file=sys.stderr)
        elif (self._colorSensor.color == self._colorSensor.COLOR_RED):
            self._actualColor = self._colorSensor.COLOR_RED
            #print("couleur (rouge) = ", self._actualColor, file=sys.stderr)
        elif (self._colorSensor.color == self._colorSensor.COLOR_WHITE):
            self._actualColor = self._colorSensor.COLOR_WHITE
            #print("couleur (blanc) = ", self._actualColor, file=sys.stderr)
        else:
            self._actualColor = self._colorSensor.COLOR_BROWN
            #print("couleur (marron) = ", self._actualColor, file=sys.stderr)

    def scanColor(self):
        if (self._actualColor != self._colorSensor.COLOR_BROWN):
            # balayage jusqu'a marron
            while (self._actualColor != self._colorSensor.COLOR_BROWN):
                # Tourne à gauche (normalement)
                self.bothMotorsRotation(10, -8, -0.05)
            print("couleur = ", self._actualColor, file=sys.stderr)
        while (self._actualColor == self._colorSensor.COLOR_BROWN):
            # Tourne à droite (normalement)
            self.bothMotorsRotation(10, -8, 0.05)
        self._leftColor = self._actualColor
        print("couleur = ", self._actualColor, file=sys.stderr)
        while (self._actualColor == self._leftColor or self._actualColor == self._colorSensor.COLOR_BROWN):
            # Tourne à droite (normalement)
            self.bothMotorsRotation(10, -8, 0.05)
        self._rightColor = self._actualColor
        print("couleur = ", self._actualColor, file=sys.stderr)
        while (self._actualColor != self._leftColor):
            self.bothMotorsRotation(10, -8, -0.05)

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

    def setIsWallAhead(self):
        while(not(self._stopThread)):
            self._isWallAhead = self._ultrasonicSensor.distance_centimeters<10
            #print(self._ultrasonicSensor.distance_centimeters, file=sys.stderr)

    def setColor(self):
        while (not (self._stopThread)):
            self.color()

def main():
    twin = RobotTwin(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4, INPUT_2)


    i=0
    x=-1
    while(i<100):
        x = random.randrange(7)
        if(x == 2):
            twin.turn180()
            twin.moveForwardOneSquare2()
        elif (x==0):
            twin.turnLeft()
            twin.moveForwardOneSquare2()
        elif (x==1):
            twin.turnRight()
            twin.moveForwardOneSquare2()
        else:
            twin.moveForwardOneSquare2()
        i+=1
        print(i, file=sys.stderr)
    
    twin._stopThread = True
    


if __name__ == '__main__':
    main()