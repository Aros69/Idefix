#!/usr/bin/env python3

import os
import sys
import time
import socket
import pickle
from ev3dev2.motor import OUTPUT_A, OUTPUT_D
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4

sys.path.append('../')

from robots.tank.robotTank import RobotTank
from robots.twins.robotTwins import RobotTwin
from RobotCommand.robotCommand import RobotCommand
from RobotCommand.moveForward import RobotMoveForward
from RobotCommand.turn180 import RobotTurn180
from RobotCommand.turnLeft import RobotTurnLeft
from RobotCommand.turnRight import RobotTurnRight

def debug_print(*args, **kwargs):
    print(*args, **kwargs, file=sys.stderr)


def main():
    # Debug print
    print('Starting Network Client Code')
    debug_print('Code client reseau lance')

    connexion = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connexion.connect(('10.42.0.1', 4242))

    # Debug print
    print('Connexion found !')
    debug_print('Connexion found !')
    
    #tank = RobotTank(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4)
    tank = RobotTwin(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4, INPUT_3)
    stopConnexion = False
    while(not(stopConnexion)):
        data = connexion.recv(4096)
        commandReceive = pickle.loads(data)
        commandReceive.setRobot(tank)
        commandReceive.doCommand()
        commandReceive = RobotCommand()
    connexion.close

if __name__ == '__main__':
    main()
