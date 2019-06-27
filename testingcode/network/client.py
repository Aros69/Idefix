#!/usr/bin/env python3

import os
import sys
import time
import socket
import pickle
from ev3dev2.motor import OUTPUT_A, OUTPUT_D
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4

sys.path.append('../')

from robots.twins.robotTwins import RobotTwin
from RobotCommand.Commands import *
from ServerCommand.Commands import *
def debug_print(*args, **kwargs):
    print(*args, **kwargs, file=sys.stderr)
class Client:
    connexion = None
    bot = None
    def __init__(self,serverIp):
        print('Starting Client...')
        self.connexion = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connexion.connect((serverIp, 4242))
        print("Successfully connected to server !")
        self.bot = RobotTwin(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4, INPUT_2)
        print("Client successfully initialized")

    def closeConnection(self):
        self.connexion.close()

    def send(self,data):
        print("sending data to server")
        data_string = pickle.dumps(commandToSend)
        if(id == "a" or id == "A"):
            self.robotASocket.send(data_string)

    def loop(self):
        print("Client Loop started")
        stopConnexion = False
        while(not(stopConnexion)):
            data = self.connexion.recv(4096)
            print("received 4096 bytes")
            commandReceive = pickle.loads(data)
            print("Command deserialized")
            commandReceive.setRobot(self.bot)
            print("Robot set")
            commandReceive.doCommand()
            print("Command ran")
            commandReceive = RobotCommand()
        
    
def main():
    # Debug print
    # print('Starting Network Client Code')
    # debug_print('Code client reseau lance')

    # connexion = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # connexion.connect(('192.168.43.203', 4242))

    # Debug print

    #bot = RobotTwin(OUTPUT_A, OUTPUT_D, INPUT_1, INPUT_4, INPUT_3)
    client = Client('192.168.43.203')
    client.loop()
    client.closeConnection()
    stopConnexion = False
    # while(not(stopConnexion)):
    #     data = connexion.recv(4096)
    #     commandReceive = pickle.loads(data)
    #     commandReceive.setRobot(bot)
    #     commandReceive.doCommand()
    #     commandReceive = RobotCommand()
    # connexion.close

if __name__ == '__main__':
    main()