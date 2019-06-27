
'''Hello to the world from ev3dev.org'''

import os
import sys
import time
import socket
import pickle
from multiprocessing import Pool, Process, Manager
from random import randint
import random

import networkx as nx
from networkx.drawing import nx_agraph
import re
import matplotlib.pyplot as plt


from command import Command
from RobotCommand.Commands import *
from ServerCommand.Commands import *

sys.path.append(os.path.realpath('../../'))
from testingcode.exploration.labyrinthe import Labyrinthe
from testingcode.exploration.directionEnum import Direction
# from testingcode.robots.tank.robotTank import RobotTank
# from testingcode.robots.twins.robotTwins import RobotTwin
from testingcode.robots.bot import bot
class Server:
    serverIp = ""
    connexion = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    robotASocket = None
    robotAInfo = None
    robotBSocket = None
    robotBInfo = None
    robotCSocket = None
    robotCInfo = None

    labyrinth = None
    robot_list = []

    def __init__(self,ip):
        self.robotTank = bot()
        self.robotTwins1 = bot()
        self.robotTwins2 = bot()

        self.robot_list.append(self.robotTank)
        self.robot_list.append(self.robotTwins1)
        self.robot_list.append(self.robotTwins2)

        self.graph_dim = (8,8)
        self.serverIp = ip
        self.connexion.bind((ip, 4242))
        self.connexion.listen(5)
        print("Server successfully initialized")

    def connectA(self):
        print("Waiting for robot A")
        self.robotASocket,self.robotAInfo = self.connexion.accept()
        print("Robot A connected")

    def connectB(self):
        print("Waiting for robot B")
        self.robotBSocket,self.robotBInfo = self.connexion.accept()
        print("Robot B connected")

    def connectC(self):
        print("Waiting for robot C")
        self.robotCSocket,self.robotCInfo = self.connexion.accept()
        print("Robot C connected")

    def initSockets(self) :
        self.connectA()
        self.connectB()
        self.connectC()

    def handleResponse(self, id, command):
        print("Received response from ", id , " \nHandling it...")
        #Take into account if response is None


    def send(self,id,commandToSend):
        data_string = pickle.dumps(commandToSend)
        if(id == "a" or id == "A"):
            print("Sending command to robot A")
            self.robotASocket.send(data_string)
            print("Sent")
        elif(id == "b" or id == "B"):
            print("Sending command to robot B")
            self.robotBSocket.send(data_string)
            print("Sent")
        elif(id == "c" or id == "C"):
            print("Sending command to robot C")
            self.robotCSocket.send(data_string)
            print("Sent")
        else :
            print("BAD ROBOT ID")

    def receive(self,id):
        if(id == "a" or id == "A"):
            print("Waiting for robot A's response")
            data = self.robotASocket.recv(4096)
            commandReceive = pickle.loads(data)

        elif(id == "b" or id == "B"):
            print("Waiting for robot B's response")
            data = self.robotBSocket.recv(4096)
            commandReceive = pickle.loads(data)
        elif(id == "c" or id == "C"):
            print("Waiting for robot C's response")
            data = self.robotCSocket.recv(4096)
            commandReceive = pickle.loads(data)
        else :
            print("BAD ROBOT ID")

    def closeSockets(self):
        self.robotASocket.close()
        self.robotBSocket.close()
        self.robotCSocket.close()
    
    def loopCommands(self):
        stopConnexion = False
        while(not(stopConnexion)):
            tmp = input()
            if("front" in tmp):
                commandToSend = RobotMoveForward()
            elif("back" in tmp):
                commandToSend = RobotTurn180()
            elif("left" in tmp):
                commandToSend = RobotTurnLeft()
            elif("right" in tmp):
                commandToSend = RobotTurnRight()
            elif("line" in tmp):
                commandToSend = RobotTurnRight()
            elif("north" in tmp):
                number = [int(s) for s in tmp.split() if s.isdigit()]
                commandToSend = RobotGoThere(0,number[0])
            elif("east" in tmp):
                number = [int(s) for s in tmp.split() if s.isdigit()]
                commandToSend = RobotGoThere(1,number[0])
            elif("south" in tmp):
                number = [int(s) for s in tmp.split() if s.isdigit()]
                commandToSend = RobotGoThere(2,number[0])
            elif("west" in tmp):
                number = [int(s) for s in tmp.split() if s.isdigit()]
                commandToSend = RobotGoThere(3,number[0])
            elif(tmp == "quit" or tmp == "exit"):
                stopConnexion = True
                commandToSend = RobotCommand()
            else:
                commandToSend = RobotCommand()
            command = [s for s in tmp.split()]
            self.send(command[0],commandToSend)
        self.closeSockets()

    '''
        :return: graph corrected and node not explored yet
    '''
    def correct_labyrinth(self, x, y, dim_x, dim_y, robot):
        laby = Labyrinthe(x, y, dim_x, dim_y, robot.get_position())

        # calculate all shortest path.
        to_visit = laby.not_visited_node()
        path_block = False

        while len(to_visit) > 0 and not path_block:
            path = laby.nearest_node(to_visit)

            # if there are a path
            if len(path) > 0:
                success = robot.robot_follow(path)

                if (success):
                    edges = self.robot_scan()
                    laby.graph.remove_edges_from(edges)
                    to_visit.remove((path[-1]))
                    laby.set_robot_pos(path[-1])
            else:
                path_block = True

        return laby.graph, to_visit


    # TODO not finish
    def run(self):
        manager = Manager()
        to_vist = manager.dict()
        proc_list = []
        laby = nx.graph()

        i = 0
        for robot in self.robot_list:
            proc_list.append(Process(target=robot.correct_labyrinth, args=(i, to_vist)))
            proc_list[i].start()
            i += 1

        not_visited = []
        for i in range (0, len(self.robot_list)):
            proc_list[i].join()
            laby = nx.union(laby, self.robot_list[i].get_labyrinthe())
            not_visited.append(to_vist[i])
        

        # second step to adjuste laby
        # robots = [self.robotTank, self.robotTwins1, self.robotTwins2]
        # while len(to_visit) > 0:
        #     short_path = []
        #     short_path_lg = self.graph_dim[0] * self.graph_dim[1]
        #     i = 0
        #     for robot in robots:
        #         if nx.has_path(self.graph, robot.get_position(), to_visit[-1]):
        #             path = nx.shortest_path(self.graph, robot.get_position(), to_visit[-1])

        #             if len(path) < short_path_lg:
        #                 short_path_lg = len(path)
        #                 short_path = path
        #                 robot_id = i
        #         else:
        #             print("ERROR : no path found , impossible case")
        #             break
                
        #         i += 1

        #     success = robots[robot_id].robot_follow(short_path)

        #     if (success):
        #         edges = self.robot_scan()
        #         laby.graph.remove_edges_from(edges)
        #         to_visit.remove((path[-1]))
        
        self.labyrinth = laby

    def move_to(self, robot_p, direction):
        pos = robot_p.get_position()
        can_go = True
        if direction == Direction.LEFT:
            next_pos = (pos[0], pos[1] -1)
            while can_go:
                if not self.labyrinth.has_edge((pos, next_pos)):
                    can_go = False
                for robot in self.robot_list:
                    if robot != robot_p and next_pos == robot.get_position():
                        can_go = False
                
                if can_go:
                    pos = next_pos
                    next_pos[1] -= 1
        elif direction == Direction.UP:
            next_pos = (pos[0] -1 , pos[1])
            while can_go:
                if not self.labyrinth.has_edge((pos, next_pos)):
                    can_go = False
                for robot in self.robot_list:
                    if robot != robot_p and next_pos == robot.get_position():
                        can_go = False
                
                if can_go:
                    pos = next_pos
                    next_pos[0] -= 1
        elif direction == Direction.RIGHT:
            next_pos = (pos[0], pos[1] + 1)
            while can_go:
                if not self.labyrinth.has_edge((pos, next_pos)):
                    can_go = False
                for robot in self.robot_list:
                    if robot != robot_p and next_pos == robot.get_position():
                        can_go = False
                
                if can_go:
                    pos = next_pos
                    next_pos[1] += 1
        elif direction == Direction.DOWN:
            next_pos = (pos[0] + 1, pos[1])
            while can_go:
                if not self.labyrinth.has_edge((pos, next_pos)):
                    can_go = False
                for robot in self.robot_list:
                    if robot != robot_p and next_pos == robot.get_position():
                        can_go = False
                
                if can_go:
                    pos = next_pos
                    next_pos[0] += 1
        else:
            print("direction not recognized") 


    def move_to_bidon(self, robot_pos, direction):
        pos = robot_pos
        can_go = True
        if direction == Direction.LEFT:
            next_pos = (pos[0], pos[1] -1)
            while can_go:
                if not self.labyrinth.has_edge((pos, next_pos)):
                    can_go = False
                for r_pos in self.robot_list_pos:
                    if next_pos == r_pos:
                        can_go = False
                
                if can_go:
                    pos = next_pos
                    next_pos[1] -= 1
        elif direction == Direction.UP:
            next_pos = (pos[0] -1 , pos[1])
            while can_go:
                if not self.labyrinth.has_edge((pos, next_pos)):
                    can_go = False
                for r_pos in self.robot_list_pos:
                    if next_pos == r_pos:
                        can_go = False
                
                if can_go:
                    pos = next_pos
                    next_pos[0] -= 1
        elif direction == Direction.RIGHT:
            next_pos = (pos[0], pos[1] + 1)
            while can_go:
                if not self.labyrinth.has_edge((pos, next_pos)):
                    can_go = False
                for r_pos in self.robot_list_pos:
                    if next_pos == r_pos:
                        can_go = False
                
                if can_go:
                    pos = next_pos
                    next_pos[1] += 1
        elif direction == Direction.DOWN:
            next_pos = (pos[0] + 1, pos[1])
            while can_go:
                if not self.labyrinth.has_edge((pos, next_pos)):
                    can_go = False
                for r_pos in self.robot_list_pos:
                    if next_pos == r_pos:
                        can_go = False
                
                if can_go:
                    pos = next_pos
                    next_pos[0] += 1
        else:
            print("direction not recognized") 

def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def main():
    '''The main function of our program'''

    # connexion = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Création du socket server
    # connexion.bind(('192.168.43.203', 4242)) # Définition du port et du nom hote
    # connexion.listen(5) # Définition du nombre de connexion en cours d'acceptation
    # debug_print('On attend un client')
    # socketClient, infoClient = connexion.accept() # Fonction d'acceptation d'un client (fonction bloquante)
    # debug_print('On a reçu un client')
    # debug_print(infoClient)
    server = Server('192.168.43.203')
    server.connectA()
    server.connectB()
    server.connectC()
    server.loopCommands()
    #server.connectA()
    # stopConnexion = False
    # while(not(stopConnexion)):
    #     tmp = input()
    #     if(tmp=="z"):
    #         commandToSend = RobotMoveForward()
    #     elif(tmp =="s"):
    #         commandToSend = RobotTurn180()
    #     elif(tmp == "q"):
    #         commandToSend = RobotTurnLeft()
    #     elif(tmp == "d"):
    #         commandToSend = RobotTurnRight()
    #     elif(tmp == "f"):
    #         commandToSend = RobotTurnRight()
    #     elif("north" in tmp):
    #         number = [int(s) for s in tmp.split() if s.isdigit()]
    #         commandToSend = RobotGoThere(0,number[0])
    #     elif("east" in tmp):
    #         number = [int(s) for s in tmp.split() if s.isdigit()]
    #         commandToSend = RobotGoThere(1,number[0])
    #     elif("south" in tmp):
    #         number = [int(s) for s in tmp.split() if s.isdigit()]
    #         # number[0] is the unit with which to turn
    #         commandToSend = RobotGoThere(2,number[0])
    #     elif("west" in tmp):
    #         number = [int(s) for s in tmp.split() if s.isdigit()]
    #         # number[0] is the unit with which to turn
    #         print()
    #         commandToSend = RobotGoThere(3,number[0])
    #     elif(tmp == "quit" or tmp == "exit"):
    #         stopConnexion = True
    #         commandToSend = RobotCommand()
    #     else:
    #         commandToSend = RobotCommand()
    #     data_string = pickle.dumps(commandToSend)
    #     socketClient.send(data_string)
    # socketClient.close()

def move_to_bidon(robot_pos, direction, laby, robot_list_pos):
    pos = robot_pos
    can_go = True
    if direction == Direction.LEFT:
        next_pos = (pos[0], pos[1] -1)
        while can_go:
            if not laby.has_edge(pos, next_pos):
                can_go = False
            for r_pos in robot_list_pos:
                if next_pos == r_pos:
                    can_go = False
            
            if can_go:
                pos = next_pos
                next_pos = (next_pos[0], next_pos[1] - 1)
    elif direction == Direction.UP:
        next_pos = (pos[0] -1 , pos[1])
        while can_go:
            if not laby.has_edge(pos, next_pos):
                can_go = False
            for r_pos in robot_list_pos:
                if next_pos == r_pos:
                    can_go = False
            
            if can_go:
                pos = next_pos
                next_pos = (next_pos[0] - 1, next_pos[1])

    elif direction == Direction.RIGHT:
        next_pos = (pos[0], pos[1] + 1)
        while can_go:
            if not laby.has_edge(pos, next_pos):
                can_go = False
            for r_pos in robot_list_pos:
                if next_pos == r_pos:
                    can_go = False
            
            if can_go:
                pos = next_pos
                next_pos = (next_pos[0], next_pos[1] + 1)
    elif direction == Direction.DOWN:
        next_pos = (pos[0] + 1, pos[1])
        while can_go:
            if not laby.has_edge(pos, next_pos):
                can_go = False
            for r_pos in robot_list_pos:
                if next_pos == r_pos:
                    can_go = False
            
            if can_go:
                pos = next_pos
                next_pos[0] += 1
                next_pos = (next_pos[0] + 1, next_pos[1])

    else:
        print("direction not recognized")

    return can_go
    

def correct_labyrinth_bidon(id, output_para, robot_pos, labyrinth, to_visit):
    robotPos = robot_pos
    laby = labyrinth

    # TODO il ne faut pas géré le cas départ ou le robot est sur le noeud avec un remove
    # TODO certe ça optimise mais pour la généricité quand le robot doit aider les autres c'est mieux.
    # calculate all shortest path.
    #to_visit = laby.not_visited_node()
    # to_visit = to_visit
    # to_visit.remove(robotPos)

    path_block = False
    
    edge_deleted = []


    while len(to_visit) > 0 and not path_block:
        # print ("to_visit = ", to_visit)
        path = laby.nearest_node(to_visit)
        

        # if there are a path (we also consider case where robot is already on correct case for generic)
        if len(path) > 0:
            # simulation of block during go to node
            if (len(path) > 1):
                success = random.choice([True,False, True, True])
                if not success:
                    node_block = random.randint(1, len(path) - 1)


            else:
                success = True

            
            if (success):
                robotPos = (path[-1])

                neighbNode = [(robotPos[0] + 1,robotPos[1]),
                                (robotPos[0] - 1,robotPos[1]),
                                (robotPos[0],robotPos[1] + 1),
                                (robotPos[0],robotPos[1] - 1)
                            ]
                edges = []
                for i in range (0, random.randrange(4)):
                    node = random.choice(neighbNode)
                    neighbNode.remove(node)
                    node = (robotPos, node)
                    edges.append(node)

                # print ('scan = ', edges)
                edge_deleted += edges #TODO deleted edge, tempo function
                laby.graph.remove_edges_from(edges)
                to_visit.remove((path[-1]))
                laby.set_robot_pos(path[-1])
            
            else:
                robotPos = (path[node_block-1])
                laby.set_robot_pos(path[node_block-1])
                laby.graph.remove_edge(path[node_block-1],  path[node_block])
                
        else:
            # print ("PATH BLOCK")
            path_block = True
    
    output_para['to_visit_' + str(id)] = to_visit
    output_para['laby_' + str(id)] = laby.graph
    output_para['deleted_edge_' + str(id)] = edge_deleted
    
    # pos = dict( (n, n) for n in laby.graph.nodes() )
    # nx.draw_networkx(laby.graph, pos = pos) 
    # plt.axis('off')
    # plt.show()



def complet_exploration():
    while len(to_visit) > 0 and not path_block:
        # print ("to_visit = ", to_visit)
        path = laby.nearest_node(to_visit)
        

        # if there are a path (we also consider case where robot is already on correct case for generic)
        if len(path) > 0:
            # simulation of block during go to node
            if (len(path) > 1):
                success = random.choice([True,False, True, True])
                if not success:
                    node_block = random.randint(1, len(path) - 1)


            else:
                success = True

            
            if (success):
                robotPos = (path[-1])

                neighbNode = [(robotPos[0] + 1,robotPos[1]),
                                (robotPos[0] - 1,robotPos[1]),
                                (robotPos[0],robotPos[1] + 1),
                                (robotPos[0],robotPos[1] - 1)
                            ]
                edges = []
                for i in range (0, random.randrange(4)):
                    node = random.choice(neighbNode)
                    neighbNode.remove(node)
                    node = (robotPos, node)
                    edges.append(node)

                # print ('scan = ', edges)
                edge_deleted += edges #TODO deleted edge, tempo function
                laby.graph.remove_edges_from(edges)
                to_visit.remove((path[-1]))
                laby.set_robot_pos(path[-1])
            
            else:
                robotPos = (path[node_block-1])
                laby.set_robot_pos(path[node_block-1])
                laby.graph.remove_edge(path[node_block-1],  path[node_block])
                
        else:
            # print ("PATH BLOCK")
            path_block = True


def main_bidon():
    ####### data robot #############
    robotTankPos = (0,0)
    robotTwins1Pos = (3,0)
    robotTwins2Pos = (3,4)
    
    manager = Manager()
    robot_para = manager.dict()
    proc_list = []
    
    laby_size = (4,4)

    # the complete graph
    laby_complet = Labyrinthe(0,0,laby_size[0],laby_size[1], (0,0))
    laby_complet.init2DGraph()
    laby = laby_complet.graph
    node_not_explored = laby_complet.not_visited_node()

    # Divide node to visit
    to_visit_r1 = node_not_explored[0:4]
    to_visit_r2 = node_not_explored[4:]

    # TODO To change
    laby_1 = Labyrinthe(0,0, 3, 3, (1,1))
    laby_1.init2DGraph()
    laby_2 = Labyrinthe(3,0, 3, 3, (3,0))
    laby_2.init2DGraph()


    proc_list.append(Process(target=correct_labyrinth_bidon, args=(0, robot_para, robotTankPos, laby_complet, to_visit_r1)))
    proc_list[0].start()
    proc_list.append(Process(target=correct_labyrinth_bidon, args=(1, robot_para, robotTwins1Pos, laby_complet, to_visit_r2)))
    proc_list[1].start()

    # exploration data of robot
    not_visited = []
    deleted_edge = []
    for i in range (0, 2):
        proc_list[i].join()
        not_visited += robot_para['to_visit_' + str(i)]
        deleted_edge += robot_para['deleted_edge_' + str(i)]
    
    laby.remove_edges_from(deleted_edge)
    laby_complet.graph.remove_edges_from(deleted_edge)


    #################### finish to complet laby ################
    print ("not finished explored" , not_visited)


    ################ SOLVER #####################
    robotTankPos = (0,1)
    robotTwins1Pos = (3,0)

    robot_list_pos = [robotTankPos, robotTwins1Pos]

    can_go = move_to_bidon(robotTankPos, Direction.LEFT, laby, robot_list_pos)
    print(can_go)
    
    pos = dict( (n, n) for n in laby.nodes() )
    nx.draw_networkx(laby, pos = pos) 
    plt.axis('off')
    plt.show()

    
    # pos = dict( (n, n) for n in robot_para['laby_1'].nodes() )
    # nx.draw_networkx(robot_para['laby_1'], pos = pos) 
    # plt.axis('off')
    # plt.show()




if __name__ == '__main__':
    main()
    #main_bidon()
