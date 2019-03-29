#!/usr/bin/env python3

import os
import sys
import time
import socket
import pickle
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_4

sys.path.append('../')
print(sys.path, file=sys.stderr)

from robots.tank.robotTank import RobotTank
from RobotCommand.moveForward import RobotMoveForward
from RobotCommand.turn180 import RobotTurn180
from RobotCommand.turnLeft import RobotTurnLeft
from RobotCommand.turnRight import RobotTurnRight


# state constants
ON = True
OFF = False


def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def reset_console():
    '''Resets the console to the default state'''
    print('\x1Bc', end='')


def set_cursor(state):
    '''Turn the cursor on or off'''
    if state:
        print('\x1B[?25h', end='')
    else:
        print('\x1B[?25l', end='')


def set_font(name):
    '''Sets the console font

    A full list of fonts can be found with `ls /usr/share/consolefonts`
    '''
    os.system('setfont ' + name)


def main():
    '''The main function of our program'''

    # set the console just how we want it
    #reset_console()
    #set_cursor(OFF)
    #set_font('Lat15-Terminus24x12')

    # Debug print
    print('Starting Network Client Code')
    debug_print('Code client reseau lance')

    connexion = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connexion.connect(('10.42.0.1', 4242))

    # Debug print
    print('Connexion found !')
    debug_print('Connexion found !')
    

    tank = RobotTank(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4)
    stopConnexion = False
    while(not(stopConnexion)):
        data = connexion.recv(4096)
        commandReceive = pickle.loads(data)
        #tmp = commandReceive.getString()
        # if(tmp=="quit"):
        #     stopConnexion=True
        # print(tmp)
        # if(tmp=="z"):
        #     tank_drive = MoveTank(OUTPUT_A, OUTPUT_D)
        #     tank_drive.on_for_rotations(SpeedPercent(50), SpeedPercent(50), 1)
        # elif(tmp =="s"):
        #     tank_drive = MoveTank(OUTPUT_A, OUTPUT_D)
        #     tank_drive.on_for_rotations(SpeedPercent(-50), SpeedPercent(-50), 1)
        # elif(tmp == "q"):
        #     tank_drive = MoveTank(OUTPUT_A, OUTPUT_D)
        #     tank_drive.on_for_rotations(SpeedPercent(-30), SpeedPercent(50), 1)
        # elif(tmp == "d"):
        #     tank_drive = MoveTank(OUTPUT_A, OUTPUT_D)
        #     tank_drive.on_for_rotations(SpeedPercent(50), SpeedPercent(-30), 1)
        commandReceive.setRobot(tank)
        commandReceive.doCommand()
    connexion.close


if __name__ == '__main__':
    main()
