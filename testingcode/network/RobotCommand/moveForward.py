#!/usr/bin/env python3

class RobotMoveForward: 
    _robot = 0

    def __init__(self):
        self._robot = 0

    def setRobot(self, robot):
        self._robot = robot

    def doCommand(self):
        self._robot.moveForwardOneSquare()