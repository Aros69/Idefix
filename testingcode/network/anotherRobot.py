#!/usr/bin/env python3

import os
import sys
import time
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank


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

def moteurGaucheRotationHoraire(puissance, nbRotation):
    m = LargeMotor(OUTPUT_A)
    m.on_for_rotations(SpeedPercent(puissance), nbRotation)
    time.sleep(1)

def moteurGaucheRotationAntiHoraire(puissance, nbRotation):
    m = LargeMotor(OUTPUT_A)
    m.on_for_rotations(SpeedPercent(-puissance), nbRotation)
    time.sleep(1)

def doubleMoteurAvancer(puissanceGauche, puissanceDroite, nbRotation):
    tank_drive = MoveTank(OUTPUT_A, OUTPUT_D)
    tank_drive.on_for_rotations(SpeedPercent(puissanceGauche), SpeedPercent(puissanceDroite), nbRotation)
    time.sleep(1)


def main():
    '''The main function of our program'''

    # set the console just how we want it
    reset_console()
    set_cursor(OFF)
    set_font('Lat15-Terminus24x12')

    # print something to the screen of the device
    print('Hello World!')

    # 1/4 tour anti-horaire presque parfait avec tank !!
    # doubleMoteurAvancer(50, -30, 1)
    # => 1/4 tour horaire = doubleMoteurAvancer(-50, 30, 1)
    
    # Déplacement d'une case (en marche arrière parfait avec le tank
    # doubleMoteurAvancer(50, 50, 2.5)
    # => déplacement une case en avant = doubleMoteurAvancer(-50, -50, 2.5)



    # print something to the output panel in VS Code
    #debug_print('Hello VS Code!')

    # wait a bit so you have time to look at the display before the program
    # exits
    #time.sleep(5)

if __name__ == '__main__':
    main()
       