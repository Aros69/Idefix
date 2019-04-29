#!/usr/bin/env python3
'''Hello to the world from ev3dev.org'''

import os
import sys
import time
import socket
import pickle
from command import Command
from RobotCommand.robotCommand import RobotCommand
from RobotCommand.moveForward import RobotMoveForward
from RobotCommand.turn180 import RobotTurn180
from RobotCommand.turnLeft import RobotTurnLeft
from RobotCommand.turnRight import RobotTurnRight


def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def main():
    '''The main function of our program'''

    connexion = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Création du socket server
    connexion.bind(('10.42.0.1', 4242)) # Définition du port et du nom hote
    connexion.listen(5) # Définition du nombre de connexion en cours d'acceptation
    debug_print('On attends un client')
    socketClient, infoClient = connexion.accept() # Fonction d'acceptation d'un client (fonction bloquante)
    debug_print('On a reçu un client')
    debug_print(infoClient)
    
    stopConnexion = False
    while(not(stopConnexion)):
        tmp = input()
        if(tmp=="z"):
            commandToSend = RobotMoveForward()
        elif(tmp =="s"):
            commandToSend = RobotTurn180()
        elif(tmp == "q"):
            commandToSend = RobotTurnLeft()
        elif(tmp == "d"):
            commandToSend = RobotTurnRight()
        elif(tmp == "quit" or tmp == "exit"):
            stopConnexion = True
            commandToSend = RobotCommand()
        else:
            commandToSend = RobotCommand()
        data_string = pickle.dumps(commandToSend)
        socketClient.send(data_string)
    socketClient.close()
    



if __name__ == '__main__':
    main()
