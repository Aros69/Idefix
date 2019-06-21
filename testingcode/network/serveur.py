#!/usr/bin/env python3
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
from RobotCommand.robotCommand import RobotCommand
from RobotCommand.moveForward import RobotMoveForward
from RobotCommand.turn180 import RobotTurn180
from RobotCommand.turnLeft import RobotTurnLeft
from RobotCommand.turnRight import RobotTurnRight

sys.path.insert(0, "D:\\Data\\Travail\\Universite\\master\\idefix")
from testingcode.exploration.labyrinthe import Labyrinthe
from testingcode.exploration.directionEnum import Direction


class Server:
    def __init__():
        self.robotTank = RobotTank()
        self.robotTwins1 = RobotTwins()
        self.robotTwins2 = RobotTwins()

        self.robot_list = []
        self.robot_list.append(self.robotTank)
        self.robot_list.append(self.robotTwins1)
        self.robot_list.append(self.robotTwins2)

        self.graph_dim = (6,6)
        self.labyrinth = nx.graph() # Empty until self.run()


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
                sucess = robot.robot_follow(path)

                if (sucess):
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

        #     sucess = robots[robot_id].robot_follow(short_path)

        #     if (sucess):
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
    
def move_to_bidon(robot_pos, direction, laby, robot_list_pos):
        pos = robot_pos
        can_go = True
        if direction == Direction.LEFT:
            next_pos = (pos[0], pos[1] -1)
            while can_go:
                if not laby.has_edge((pos, next_pos)):
                    can_go = False
                for r_pos in robot_list_pos:
                    if next_pos == r_pos:
                        can_go = False
                
                if can_go:
                    pos = next_pos
                    next_pos[1] -= 1
        elif direction == Direction.UP:
            next_pos = (pos[0] -1 , pos[1])
            while can_go:
                if not laby.has_edge((pos, next_pos)):
                    can_go = False
                for r_pos in robot_list_pos:
                    if next_pos == r_pos:
                        can_go = False
                
                if can_go:
                    pos = next_pos
                    next_pos[0] -= 1
        elif direction == Direction.RIGHT:
            next_pos = (pos[0], pos[1] + 1)
            while can_go:
                if not laby.has_edge((pos, next_pos)):
                    can_go = False
                for r_pos in robot_list_pos:
                    if next_pos == r_pos:
                        can_go = False
                
                if can_go:
                    pos = next_pos
                    next_pos[1] += 1
        elif direction == Direction.DOWN:
            next_pos = (pos[0] + 1, pos[1])
            while can_go:
                if not laby.has_edge((pos, next_pos)):
                    can_go = False
                for r_pos in robot_list_pos:
                    if next_pos == r_pos:
                        can_go = False
                
                if can_go:
                    pos = next_pos
                    next_pos[0] += 1
        else:
            print("direction not recognized")

def correct_labyrinth_bidon(id, output_para, robot_pos, labyrinth, to_visit):
    robotPos = robot_pos
    laby = labyrinth

    # TODO il ne faut pas géré le cas départ ou le robot est sur le noeud avec un remove
    # TODO certe ça optimise mais pour la généricité quand le robot doit aider les autres c'est mieux.
    # calculate all shortest path.
    #to_visit = laby.not_visited_node()
    to_visit = to_visit
    # to_visit.remove(robotPos)
    path_block = False
    
    edge_deleted = []


    while len(to_visit) > 0 and not path_block:
        # print ("to_visit = ", to_visit)
        path = laby.nearest_node(to_visit)
        
        # print("path = ", path)

        # if there are a path
        if len(path) > 0:
            # sucess = random.choice([True,False])
            sucess = True

            if (sucess):
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
            # print ("PATH BLOCK")
            path_block = True
    
    output_para['to_visit_' + str(id)] = to_visit
    output_para['laby_' + str(id)] = laby.graph
    output_para['deleted_edge_' + str(id)] = edge_deleted
    
    # pos = dict( (n, n) for n in laby.graph.nodes() )
    # nx.draw_networkx(laby.graph, pos = pos) 
    # plt.axis('off')
    # plt.show()


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
    print (node_not_explored)
    print ("robot 1 visit ")
    print (to_visit_r1)
    print ("robot 2 visit ")
    print (to_visit_r2)

    # TODO To change
    laby_1 = Labyrinthe(0,0, 3, 3, (1,1))
    laby_1.init2DGraph()
    laby_2 = Labyrinthe(3,0, 3, 3, (3,0))
    laby_2.init2DGraph()


    proc_list.append(Process(target=correct_labyrinth_bidon, args=(0, robot_para, robotTankPos, laby_complet, to_visit_r1)))
    proc_list[0].start()
    proc_list.append(Process(target=correct_labyrinth_bidon, args=(1, robot_para, robotTwins1Pos, laby_complet, to_visit_r2)))
    proc_list[1].start()

    not_visited = []
    deleted_edge = []
    for i in range (0, 2):
        proc_list[i].join()
        not_visited.append(robot_para['to_visit_' + str(i)])
        deleted_edge += robot_para['deleted_edge_' + str(i)]
    print (not_visited)

    # for i in range()
    # laby = nx.union(laby_1.graph, laby_2.graph)
    # laby = nx.union(robot_para['laby_0'], robot_para['laby_1'])
    laby.remove_edges_from(deleted_edge)
    # print (laby.nodes())
    
    pos = dict( (n, n) for n in laby.nodes() )
    nx.draw_networkx(laby, pos = pos) 
    plt.axis('off')
    plt.show()

    
    # pos = dict( (n, n) for n in robot_para['laby_1'].nodes() )
    # nx.draw_networkx(robot_para['laby_1'], pos = pos) 
    # plt.axis('off')
    # plt.show()

    ################ SOLVER #####################
    # robotTankPos = (0,1)
    # robotTwins1Pos = (3,0)

    # robot_list_pos = [robotTankPos, robotTwins1Pos]

    # move_to_bidon(robotTankPos, Direction.LEFT, laby, robot_list_pos)
    # print(robot_list_pos)



if __name__ == '__main__':
    # main()
    main_bidon()
