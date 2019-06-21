import sys

class RobotCommand: 
    _robot = 0

    def __init__(self):
        _robot = 0

    def setRobot(self, robot):
        self._robot = robot

    def doCommand(self):
        print("Commande vide", file=sys.stderr)
