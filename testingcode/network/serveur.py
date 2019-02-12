#!/usr/bin/env python3
'''Hello to the world from ev3dev.org'''

import os
import sys
import time
import socket
import pickle
from command import Command


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
    
    commandToSend = Command()
    stopConnexion = False
    while(not(stopConnexion)):
        tmp = input()
        if(tmp == "quit"):
            stopConnexion = True
        commandToSend.setString(tmp)
        data_string = pickle.dumps(commandToSend)
        socketClient.send(data_string)
    socketClient.close()
    



if __name__ == '__main__':
    main()
