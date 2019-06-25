#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_C, OUTPUT_B, SpeedRPM, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4, Sensor
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
import os
import sys
import time
import random
from threading import Timer, Thread, Event


# sys.path.insert(0, "/your/path/to/idefix") # if you want to execute main in this file
# sys.path.insert(0, "/home/baoanh/Travail/Master1/idefix/")
#sys.path.insert(0, "/home/robin/Master/IDEFIX/idefix/")
#from testingcode.exploration.labyrinthe import Labyrinthe

class RobotTank:
    # DATA
    _motors = None
    _leftMotor = None
    _rightMotor = None
    _leftMotorPower = 16
    _rightMotorPower = 16
    _colorSensor = None
    _ultrasonicSensor = None
    _compassSensor = None
    _isWallAhead = True
    _actualColor = 0
    _rightColor = 0
    _leftColor = 0
    _threadSonic = None
    _threadColor = None
    _threadCorrection = None
    _stopCorThread = False
    _stopThread = False
    _labyrinthe = None
    _position = None  # Tuple (i,j)

    # METHODS
    def __init__(self, leftMotor, rightMotor, colorSensor, ultrasonicSensor, compassSensor, position, labyrinthe):
        self._motors = MoveTank(leftMotor, rightMotor)
        self._leftMotor = LargeMotor(leftMotor)
        self._rightMotor = LargeMotor(rightMotor)
        self._colorSensor = ColorSensor(colorSensor)
        self._ultrasonicSensor = UltrasonicSensor(ultrasonicSensor)
        self._compassSensor = Sensor(INPUT_2)
        time.sleep(1)
        self._baseAngle = self._compassSensor.value()
        self._actualAngle = self._baseAngle
        self._threadSonic = Thread(target=self.setIsWallAhead, args=[])
        self._threadColor = Thread(target=self.setColor, args=[])
        self._threadCorrection = Thread(target=self.setCorrection, args=[])
        self._threadSonic.start()
        self._threadColor.start()
        time.sleep(1)
        self._position = position
        self._rightColor = 1
        self._leftColor = 5
        # self._labyrinthe = Labyrinthe(0, 0, 8,8, position)
        # self._labyrinthe = labyrinthe

        #self._labyrinthe.init2DGraph()

    def turnLeft(self):
        angleObjectif = (self._baseAngle + 90) % 360
        self.bothMotorsRotation(30, -30, -0.85)
        time.sleep(1)
        self._actualAngle = self._compassSensor.value()
        self._baseAngle = angleObjectif
        self.orientationCorrection()

    def turnRight(self):
        angleObjectif = (self._baseAngle - 90) % 360
        self.bothMotorsRotation(30, -30, 0.85)
        time.sleep(1)
        self._actualAngle = self._compassSensor.value()
        self._baseAngle = angleObjectif
        self.orientationCorrection()

    def turn180(self):
        angleObjectif = (self._baseAngle - 180) % 360
        self.bothMotorsRotation(30, -30, 1.7)
        time.sleep(1)
        self._actualAngle = self._compassSensor.value()
        self._baseAngle = angleObjectif
        self.orientationCorrection()

    def moveForwardOneSquare(self):
        if (not (self._isWallAhead)):
            # Puisance moteur droit légerement moins puissant pour compenser un soucis (48 peut être)
            self.bothMotorsRotation(50, 50, 2.5)
            self.orientationCorrection()

    def bothMotorsRotation(self, leftPuissance, rightPuissance, rotation):
        self._motors.on_for_rotations(SpeedPercent(leftPuissance), SpeedPercent(rightPuissance), rotation)
        time.sleep(1)

    def stopMotors(self):
        self._motors.off()

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
                self.runForever(-10, 10)
                # self.bothMotorsRotation(12, -12, -0.05)
            self.stopMotors()
            print("couleur = ", self._actualColor, file=sys.stderr)
        while (self._actualColor == self._colorSensor.COLOR_BROWN):
            # Tourne à droite (normalement)
            self.runForever(10, -10)
            # self.bothMotorsRotation(12, -12, 0.05)
        self.stopMotors()
        self._leftColor = self._actualColor
        print("couleur = ", self._actualColor, file=sys.stderr)
        while (self._actualColor == self._leftColor or self._actualColor == self._colorSensor.COLOR_BROWN):
            # Tourne à droite (normalement)
            self.runForever(10, -10)
            # self.bothMotorsRotation(12, -12, 0.05)
        self.stopMotors()
        self._rightColor = self._actualColor
        print("couleur = ", self._actualColor, file=sys.stderr)
        while (self._actualColor != self._leftColor):
            self.runForever(-10, 10)
        self.stopMotors()
        # self.bothMotorsRotation(12, -12, -0.05)

    # Ultra sonique sensor detect distance from object
    def ultrasonicDetect(self):
        while (self._ultrasonicSensor.distance_centimeters > 20):
            print()
        self.runForeverAbsorbEnergy()
        self.stopMotors()

    # Motors will run until it dies or receive command to stop
    def runForever(self, leftPower, rightPower):
        self._motors.run_forever()
        self._motors.on(leftPower, rightPower)

    # Allow the robot to do a little backward avoiding to hit the wall
    def runForeverAbsorbEnergy(self):
        self._motors.run_forever()
        self._motors.on(-8, -8)

    def correctionWithColor(self):
        # run this code in a thread ?
        # always follow leftcolor
        # if BROWN is detected -> robot deviate on left, so modify power of motors to deviate on right
        # if rightcolor is detected (== actualcolor), so modify power of motors to deviate on left
        if(self._actualColor == self._leftColor):
            print("I am following the left color.", file=sys.stderr)
            self.setMotorsForward()
        else:
            if(self._actualColor == self._rightColor):
                while(self._actualColor == self._rightColor):
                    # turn litle left
                    print("I found the right color and so correcting to left.", file=sys.stderr)
                    self.setMotorsLeftCor()                
                self.setMotorsForward()
            else:
                while(self._actualColor == self._colorSensor.COLOR_BROWN):
                    # turn right 
                    print("I found BROWN and so correcting to right.", file=sys.stderr)
                    self.setMotorsRightCor()
                self.setMotorsForward()

    def orientationCorrection(self):
        time.sleep(0.2)
        angleDif = self._baseAngle - self._compassSensor.value()
        if (angleDif < -1 or angleDif > 1):
            while (self._compassSensor.value() < self._baseAngle - 1 or self._compassSensor.value() > self._baseAngle + 1):
                if (self._compassSensor.value() > self._baseAngle and self._compassSensor.value() < self._baseAngle + 180 % 360):
                    self.bothMotorsRotation(6, -6, 4 / 146)
                elif (self._compassSensor.value() < self._baseAngle and self._compassSensor.value() > self._baseAngle - 180 % 360):
                    self.bothMotorsRotation(6, -6, -4 / 146)
                else:
                    # print("Je sais pas quoi faire", file=sys.stderr)
                    self.bothMotorsRotation(6, -6, -4 / 146)
                # print("objectif = ", self._baseAngle, " actuel = ", self._compassSensor.value(), file=sys.stderr)
                time.sleep(0.5)

    def setColor(self):
        while (not (self._stopThread)):
            self.color()

    def setIsWallAhead(self):
        while (not (self._stopThread)):
            self._isWallAhead = self._ultrasonicSensor.distance_centimeters < 10

    def setCorrection(self):
        while(not (self._stopCorThread)):
            self.correctionWithColor()

    def runCorrectionThread(self):
        self._threadCorrection.start()
    
    def stopCorrectionThread(self):
        self._stopCorThread = True
    
    def setMotorsRunning(self):
        self._motors.run_forever()
        self._motors.on(self._leftMotorPower, self._rightMotorPower)

    def setMotorsScanLeft(self):
        self._leftMotorPower = -6
        self._rightMotorPower = 6
    
    def setMotorsScanRight(self):
        self._leftMotorPower = 6
        self._rightMotorPower = -6
    
    def setMotorsForward(self):
        self._leftMotorPower = 16
        self._rightMotorPower = 16
    
    def setMotorsLeftCor(self):
        # deviate to left
        self._leftMotorPower = 18
        self._rightMotorPower = 14
    
    def setMotorsRightCor(self):
        self._leftMotorPower = 14
        self._rightMotorPower = 18
    


    '''
    robot will try to follow the path given. if block then update labyrinth

    :param: path a array of node
    :return: true if final node achieved else false
    '''

    def robot_follow(self, path):
        # Loop throught path Attention, path contain actual position of robot
        # Get direction of each next node by using self._labyrinthe.direction()
        # Move robot
        # if block, update position of robot in self._labyrinthe
        # if block, remove edge using self._labyrinthe.graph.remove_edge()
        # if block, return False

        # if not block, update position of robot
        # if not block, return True

        return True

    '''
    robot will check if the edge around is blocked
    :return: list of not accesible edge, else empty list
    '''

    def robot_scan(self):
        # do a 360 to check if edge exist (no wall)
        # if not exist add to edges list, format [( (0,0), (0,1) ), ( (0,0), (1,1) ), ...]
        return []

    # TODO à vérifié avec l'algo bidon
    '''
    :return: True if labyrinthe corrected else other robot have to help
    '''

    def correct_labyrinth(self):

        # calculate all shortest path.
        to_visit = self._labyrinthe.not_visited_node()
        path_block = False

        while len(to_visit) > 0 and not path_block:
            path = self._labyrinthe.nearest_node(to_visit)

            # if there are a path
            if len(path) > 0:
                sucess = self.robot_follow(path)

                if (sucess):
                    edges = self.robot_scan()
                    self._labyrinthe.graph.remove_edges_from(edges)
                    to_visit.remove((path[-1]))
            else:
                path_block = True

        # to_visit is now a list of not achieved path that other robot need to check
        return not path_block, to_visit

        # transforme into robot direction mouvement
        # if chemin don't exist.
        # update graph then loop
        # graph.remove_edge

        # if destinations.
        # scan, update graph
        # remove node from not visited, then check for next shortest path.

    def get_position(self):
        return self._position

    def get_labyrinthe(self):
        return self._labyrinthe


def main():
    twin = RobotTank(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4, INPUT_2, (0, 0), None)

    #twin.scanColor()
    twin._leftMotorPower = 16
    twin._rightMotorPower = 16
    twin.runCorrectionThread()
    while(True):
        twin.setMotorsRunning()

    """ i=0
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
        print(i, file=sys.stderr) """

    twin._stopThread = True
    print("End of the fucking program !", file=sys.stderr)


if __name__ == '__main__':
    main()
