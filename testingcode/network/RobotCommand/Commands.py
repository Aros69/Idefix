#!/usr/bin/env python3
import sys

class RobotCommand: 
    _robot = 0

    def __init__(self):
        _robot = 0

    def setRobot(self, robot):
        self._robot = robot

    def doCommand(self):
        print("Commande vide", file=sys.stderr)

class RobotGoThere: 
    _robot = 0
    _targetOrientation = 0
    _units = 0
    def __init__(self,targetOrientation,units):
        self._robot = 0
        self._targetOrientation = targetOrientation
        self._units = units

    def setRobot(self, robot):
        self._robot = robot

    def doCommand(self):
        print("Go there")
        return self._robot.goThere(self._targetOrientation,self._units)

class RobotScan: 
    _robot = 0
    _targetOrientation = 0
    _units = 0
    def __init__(self,targetOrientation,units):
        self._robot = 0
        self._targetOrientation = targetOrientation
        self._units = units

    def setRobot(self, robot):
        self._robot = robot

    def doCommand(self):
        print("Go there")
        return self._robot.scan()

class RobotExecuteCommands: 
    _robot = 0
    _commands = []
    def __init__(self,commands):
        self._robot = 0
        self._commands = commands

    def setRobot(self, robot):
        self._robot = robot

    def doCommand(self):
        print("sent Command List")
        #TODO
        #self._robot.executeCommands(self._commands)

class RobotMoveForward: 
    _robot = 0

    def __init__(self):
        self._robot = 0

    def setRobot(self, robot):
        self._robot = robot

    def doCommand(self):
        print("sent move forward command")
        return self._robot.moveForwardOneSquare2()
        

class RobotTurn180: 
    _robot = 0

    def __init__(self):
        self._robot = 0

    def setRobot(self, robot):
        self._robot = robot

    def doCommand(self):
        print("sent turn 180 command")
        return self._robot.turn180()

class RobotTurnLeft: 
    _robot = 0

    def __init__(self):
        self._robot = 0

    def setRobot(self, robot):
        self._robot = robot

    def doCommand(self):
        print("sent turn left command")
        return self._robot.turnLeft()

class RobotTurnRight: 
    _robot = 0

    def __init__(self):
        self._robot = 0

    def setRobot(self, robot):
        self._robot = robot

    def doCommand(self):
        print("sent turn right command")
        return self._robot.turnRight()