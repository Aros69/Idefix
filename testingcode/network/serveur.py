
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
import copy

from command import Command
from RobotCommand.Commands import *
from ServerCommand.Commands import *


sys.path.append(os.path.realpath('../../'))
from testingcode.exploration.labyrinthe import Labyrinthe
from testingcode.exploration.directionEnum import Direction
from testingcode.solver.maze import Pos
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
        
    def __init__(self,ip,dim_x, dim_y, robotPos, nb_robot = 3, simulation = False):
        print("init maze data")
        # self.robot_0_pos = robotPos[0]
        # self.robot_1_pos = robotPos[1]
        # self.robot_2_pos = robotPos[2]

        # self.robot_pos_list = [self.robot_0_pos, self.robot_1_pos, self.robot_2_pos]
        self.simulation = simulation
        self.nb_robot = nb_robot
        self.robot_pos_list = robotPos

        
        self.arrive = (2,2)

        self.graph_dim = (dim_x,dim_y)
        self.labyrinth = Labyrinthe(0,0,dim_x,dim_y, (0,0)) # position robot not
        self.labyrinth.init2DGraph()
        print("Maze successfully initialized !")
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
        print("Response = ", command)
        #Take into account if response is None


    def send(self,id,commandToSend):
        data_string = pickle.dumps(commandToSend)
        if(id == 0):
            print("Sending command to robot A")
            self.robotASocket.send(data_string)
            print("Sent")
        elif(id == 1):
            print("Sending command to robot B")
            self.robotBSocket.send(data_string)
            print("Sent")
        elif(id == 2):
            print("Sending command to robot C")
            self.robotCSocket.send(data_string)
            print("Sent")
        else :
            print("BAD ROBOT ID")

    def receive(self,id):
        if(id == 0):
            print("Waiting for robot A's response")
            data = self.robotASocket.recv(4096)
            commandReceive = pickle.loads(data)
            self.handleResponse(id,commandReceive)
            return commandReceive
        elif(id == 1):
            print("Waiting for robot B's response")
            data = self.robotBSocket.recv(4096)
            commandReceive = pickle.loads(data)
            self.handleResponse(id,commandReceive)
            return commandReceive
        elif(id == 2):
            print("Waiting for robot C's response")
            data = self.robotCSocket.recv(4096)
            commandReceive = pickle.loads(data)
            self.handleResponse(id,commandReceive)
            return commandReceive
        else :
            print("BAD ROBOT ID")
            return None

    def closeSockets(self):
        self.robotASocket.close()
        self.robotBSocket.close()
        self.robotCSocket.close()

    def sendCommand(self,id,command):
        self.send(id,command)
        commandStatus = self.receive(id)
        return commandStatus
        print("Sending Commands")

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
                commandToSend = RobotGoThere(0,number[1])
            elif("east" in tmp):
                number = [int(s) for s in tmp.split() if s.isdigit()]
                commandToSend = RobotGoThere(1,number[1])
            elif("south" in tmp):
                number = [int(s) for s in tmp.split() if s.isdigit()]
                commandToSend = RobotGoThere(2,number[1])
            elif("west" in tmp):
                number = [int(s) for s in tmp.split() if s.isdigit()]
                commandToSend = RobotGoThere(3,number[1])
            elif(tmp == "quit" or tmp == "exit"):
                stopConnexion = True
                commandToSend = RobotCommand()
            else:
                commandToSend = RobotCommand()
            command = [int(s) for s in tmp.split() if s.isdigit()]
            self.send(command[0],commandToSend)
            self.receive(command[0])
        self.closeSockets()

    '''
        robot is a pair of position for moment
        :return: graph corrected and node not explored yet
    '''
    def correct_labyrinth(self,id, robot, to_visit, laby, output_para):
        laby.set_robot_pos(robot) #socket get position or start position
        edges_deleted = []

        # calculate all shortest path.
        path_block = False

        while len(to_visit) > 0 and not path_block:
            path = laby.nearest_node(to_visit, True)


            # if there are a path (we also consider case where robot is already on correct case for generic)
            if len(path) > 0:
                
                print ('le robot ', id, "en position: ", path[0], " essaie d aller en: ", laby.get_robot_pos())

                success,laby, temp = self.try_to_go(id, path, laby)
                edges_deleted += temp
                if success:
                    to_visit.remove(path[-1])
            else:
                print ('le robot ', id, "en position: ", laby.get_robot_pos(), " ne trouve pas chemin")
                path_block = True

        output_para['to_visit_' + str(id)] = to_visit
        output_para['laby_' + str(id)] = laby.graph
        output_para['deleted_edge_' + str(id)] = edges_deleted
        output_para['robot_pos_' + str(id)] = laby.get_robot_pos()


    '''
        we suppose self.robot_list_pos is updated and will be updated
    '''
    def correct_labyrinth_final(self, to_visit):
        path_block = False # Path block should never be true
        while len(to_visit) > 0 and not path_block:

            # find nearest node between 3 robots
            self.labyrinth.set_robot_pos(self.robot_pos_list[0])
            path = None
            current_robot = 0
            for i in range(0, self.nb_robot):
                self.labyrinth.set_robot_pos(self.robot_pos_list[i])
                new_path = self.labyrinth.nearest_node(to_visit)
                
                if (path == None or len(new_path) < len(path)) and len(new_path) > 0 :
                    path = new_path
                    current_robot = i
            
            # if there are a path (we also consider case where robot is already on correct case for generic)
            if path is not None and len(path) > 0:
                success, self.labyrinth, bidon = self.try_to_go(current_robot, path, self.labyrinth)
                
                # update position
                self.robot_pos_list[current_robot] = self.labyrinth.get_robot_pos()

                if success:
                    to_visit.remove(path[-1])
            else:
                path_block = True
        


    '''
        make robot try to follow a path
        :return: success, updated laby, edges deleted
    '''
    def try_to_go(self, id, path, laby):
        success = None
        edges_deleted = []

        if self.simulation:
            if (len(path) > 1):
                success = random.choice([True,False, True, True])
                if not success:
                    node_block = random.randint(1, len(path) - 1)
            else:
                success = True
        else:
            commands = laby.path2Command(path)
        
            r_path_indice = 0
            for command in commands :
                status = self.sendCommand(id,RobotGoThere(command[0],command[1]))
                if(status._dataId == -1): #Fail
                    success = False
                elif(status._dataId == 0): #Success
                    print("")
                    r_path_indice += 1
                    success = True

        ### robot successfully achieved path
        if (success):
            laby.set_robot_pos(path[-1]) # set robot to last path
            

            #### simumation of scaning
            if self.simulation:
                neighbNode = [(laby.robot_pos[0] + 1,laby.robot_pos[1]),
                                (laby.robot_pos[0] - 1,laby.robot_pos[1]),
                                (laby.robot_pos[0],laby.robot_pos[1] + 1),
                                (laby.robot_pos[0],laby.robot_pos[1] - 1)
                            ]
                for i in range (0, random.randrange(4)):
                    node = random.choice(neighbNode)
                    neighbNode.remove(node)
                    node = (laby.robot_pos, node)
                    edges_deleted.append(node)
            else:
                status = self.sendCommand(id,RobotGoThere(command[0],command[1]))
                for wall in status._stateWalls:
                    if wall[1] == True:
                        edge = laby.edge_by_direction(path[-1], wall[0])
                        edges_deleted.append(edge)

            print('le robot ', id, 'a reussi a aller en: ' , path[-1])
            print('le robot ', id, 'scan et trouve les mur: ', edges_deleted)

            laby.graph.remove_edges_from(edges_deleted)
            laby.cut_graph.remove_edges_from(edges_deleted)
        
        ### robot get blocked during moving
        else:
            temp = None
            if self.simulation:
                laby.set_robot_pos(path[node_block-1])
                edges_deleted.append((path[node_block-1], path[node_block-1]))
                temp = (path[node_block-1], path[node_block-1])
            else:
                laby.set_robot_pos(path[r_path_indice])
                edges_deleted.append((path[r_path_indice],  path[r_path_indice+1]))
                temp = (path[r_path_indice],  path[r_path_indice+1])

            print('le robot ', id, 'est bloquer, sa position est ' , path[-1])
            print('le robot ', id, 'a devant lui le mur ' , temp)
            laby.graph.remove_edges_from(edges_deleted)
            laby.cut_graph.remove_edges_from(edges_deleted)


        return success, laby, edges_deleted

    # TODO not finish
    def run(self):
        # threads data
        manager = Manager()
        robot_para = manager.dict()
        proc_list = []

        # list of labyrinth that robot need to explore
        laby_to_explore_0 = copy.deepcopy(self.labyrinth)
        laby_to_explore_1 = copy.deepcopy(self.labyrinth)
        laby_to_explore_2 = copy.deepcopy(self.labyrinth)
        laby_to_explore_list = [laby_to_explore_0, laby_to_explore_1, laby_to_explore_2]

        # divide node to visite for each robot + cut labyrinthe
        node_not_explored = self.labyrinth.not_visited_node()

        allnodesDiviseBy3 = int(len(node_not_explored)/self.nb_robot)
        to_visit_list = []
        for i in range (0, self.nb_robot):
            s = i*allnodesDiviseBy3
            if i+1 > 2:
                e = len(node_not_explored)
            else:
                e = (i+1)*allnodesDiviseBy3
            to_visit_list.append(node_not_explored[s:e])
            laby_to_explore_list[i].init_cut_graph(node_not_explored[s],node_not_explored[e-1])
        

        if self.nb_robot > 0:
            proc_list.append(Process(target=self.correct_labyrinth, args=(0, self.robot_pos_list[0], to_visit_list[0], laby_to_explore_list[0], robot_para)))
            proc_list[0].start()
        if self.nb_robot > 1:
            proc_list.append(Process(target=self.correct_labyrinth, args=(1, self.robot_pos_list[1], to_visit_list[1], laby_to_explore_list[1], robot_para)))
            proc_list[1].start()
        if self.nb_robot > 2:
            proc_list.append(Process(target=self.correct_labyrinth, args=(2, self.robot_pos_list[2], to_visit_list[2], laby_to_explore_list[2], robot_para)))
            proc_list[2].start()

        # exploration data of robot
        to_visit = []
        deleted_edge = []
        for i in range (0, self.nb_robot):
            proc_list[i].join()
        for i in range (0,self.nb_robot):
            to_visit += robot_para['to_visit_' + str(i)]
            deleted_edge += robot_para['deleted_edge_' + str(i)]
            self.robot_pos_list[i] = robot_para['robot_pos_' + str(i)]
        
        self.labyrinth.graph.remove_edges_from(deleted_edge)


        print ('fin des threads, les nodes suivant ne sont pas encore visite: ', to_visit)
        print ('lancement du phase 2 de completion avec 1 robot qui explore ')
        ########## finish to complet labyrinth ##############
        self.labyrinth.init_cut_graph((0,0), (self.graph_dim))
        # update robot pos (socket or maybe already update with correct)
        self.correct_labyrinth_final(to_visit)

        print ("fin de la phase exploration, le labyrinthe ressemble a ca \n \n")
        pos = dict( (n, n) for n in self.labyrinth.graph.nodes() )
        nx.draw_networkx(self.labyrinth.graph, pos = pos) 
        plt.axis('off')
        plt.show()

    def solve(self, robotPos, arrive):
        ############# Solver
        robot_pos_list_s = robotPos
        pos = Pos(((robot_pos_list_s[2][0]*8+robot_pos_list_s[2][1])* 64 
        + robot_pos_list_s[1][0]*8+robot_pos_list_s[1][1])* 64 
        + robot_pos_list_s[0][0] * 8 + robot_pos_list_s[0][1], [])
        
        robots_pos = self.solve_conf(self.labyrinth.graph, pos, arrive[0] * 8 + arrive[1])

        if robots_pos == None:
            print ("impossible")
        else:
            print("position final des robots: ", robots_pos.loc)
            print ("liste des mouvements ")
            print (robots_pos.move)

        ######## drawing ##############
        pos = dict( (n, n) for n in self.labyrinth.graph.nodes() )
        nx.draw_networkx(self.labyrinth.graph, pos = pos) 
        plt.axis('off')
        plt.show()


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


    def solve_conf(self, lab, pos, end: int):
        todo = [pos]
        visit = {pos.loc}
        while len(todo) > 0:
            current = todo[0]
            del todo[0]
            for i in range(12):
                tmp = current.loc // (64**(i//4)) % 64
                pos_r0 = (tmp // 8, tmp % 8)
                tmp = current.loc // (64**((1+i//4)%3)) % 64
                pos_r1 = (tmp // 8, tmp % 8)
                tmp = current.loc // (64**((2+i//4)%3)) % 64
                pos_r2 = (tmp // 8, tmp % 8)
                
                pos_r0, bidon = move_to_bidon(pos_r0, i%4, lab, [pos_r0, pos_r1, pos_r2])

                new = copy.deepcopy(current)
                tmp = pos_r0[0] * 8 + pos_r0[1]
                new.loc = tmp * 64 ** (i//4)
                tmp = pos_r1[0] * 8 + pos_r1[1]
                new.loc += tmp * 64 ** ((i//4 + 1) % 3)
                tmp = pos_r2[0] * 8 + pos_r2[1]
                new.loc += tmp * 64 ** ((i//4 + 2) % 3)
                new.move.append(i)
                

                if not(new.loc in visit):
                    if new.loc % 64 == end:
                        return new
                    todo.append(new)
                    visit.add(new.loc)
        return


def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def main():
    '''The main function of our program'''

    server = Server('192.168.43.203')
    server.connectA()
    server.connectB()
    server.connectC()
    server.loopCommands()
    

def move_to_bidon(robot_pos, direction, laby, robot_list_pos):
    pos = robot_pos
    list_mvt = [pos]    
    can_go = True
    if direction == 2:
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
                list_mvt.append(pos)
    elif direction == 1:
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
                list_mvt.append(pos)
    elif direction == 0:
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
                list_mvt.append(pos)

    elif direction == 3:
        next_pos = (pos[0] + 1, pos[1])
        while can_go:
            if not laby.has_edge(pos, next_pos):
                can_go = False
            for r_pos in robot_list_pos:
                if next_pos == r_pos:
                    can_go = False
            
            if can_go:
                pos = next_pos
                next_pos = (next_pos[0] + 1, next_pos[1])
                list_mvt.append(pos)

    else:
        print("direction not recognized")

    return pos, list_mvt


'''
    solution oreste avec direction en fonction du solver
'''
# def move_to_bidon(robot_pos, direction, laby, robot_list_pos):
#     pos = robot_pos
#     list_mvt = [pos]    
#     can_go = True
#     if direction == 3:
#         next_pos = (pos[0], pos[1] -1)
#         while can_go:
#             if not laby.has_edge(pos, next_pos):
#                 can_go = False
#             for r_pos in robot_list_pos:
#                 if next_pos == r_pos:
#                     can_go = False
            
#             if can_go:
#                 pos = next_pos
#                 next_pos = (next_pos[0], next_pos[1] - 1)
#                 list_mvt.append(pos)
#     elif direction == 2:
#         next_pos = (pos[0] -1 , pos[1])
#         while can_go:
#             if not laby.has_edge(pos, next_pos):
#                 can_go = False
#             for r_pos in robot_list_pos:
#                 if next_pos == r_pos:
#                     can_go = False
            
#             if can_go:
#                 pos = next_pos
#                 next_pos = (next_pos[0] - 1, next_pos[1])
#                 list_mvt.append(pos)
#     elif direction == 1:
#         next_pos = (pos[0], pos[1] + 1)
#         while can_go:
#             if not laby.has_edge(pos, next_pos):
#                 can_go = False
#             for r_pos in robot_list_pos:
#                 if next_pos == r_pos:
#                     can_go = False
            
#             if can_go:
#                 pos = next_pos
#                 next_pos = (next_pos[0], next_pos[1] + 1)
#                 list_mvt.append(pos)

#     elif direction == 0:
#         next_pos = (pos[0] + 1, pos[1])
#         while can_go:
#             if not laby.has_edge(pos, next_pos):
#                 can_go = False
#             for r_pos in robot_list_pos:
#                 if next_pos == r_pos:
#                     can_go = False
            
#             if can_go:
#                 pos = next_pos
#                 next_pos = (next_pos[0] + 1, next_pos[1])
#                 list_mvt.append(pos)

#     else:
#         print("direction not recognized")

#     return pos, list_mvt
    

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
                edge_deleted.append((path[node_block-1],  path[node_block]))
                
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


def solve_conf_bidon(lab, pos, end: int):
    todo = [pos]
    visit = {pos.loc}
    while len(todo) > 0:
        # print("loop")
        current = todo[0]
        del todo[0]
        # print(current.move)
        for i in range(12):
            # print(i)
            tmp = current.loc // (64**(i//4)) % 64
            pos_r0 = (tmp // 8, tmp % 8)
            tmp = current.loc // (64**((1+i//4)%3)) % 64
            pos_r1 = (tmp // 8, tmp % 8)
            tmp = current.loc // (64**((2+i//4)%3)) % 64
            pos_r2 = (tmp // 8, tmp % 8)
            
            pos_r0, bidon = move_to_bidon(pos_r0, i%4, lab, [pos_r0, pos_r1, pos_r2])

            new = copy.deepcopy(current)
            tmp = pos_r0[0] * 8 + pos_r0[1]
            new.loc = tmp * 64 ** (i//4)
            tmp = pos_r1[0] * 8 + pos_r1[1]
            new.loc += tmp * 64 ** ((i//4 + 1) % 3)
            tmp = pos_r2[0] * 8 + pos_r2[1]
            new.loc += tmp * 64 ** ((i//4 + 2) % 3)
            new.move.append(i)
            
            # print(current.move)
            # print(pos_r0)

            if not(new.loc in visit):
                if new.loc % 64 == end:
                    # print("success")
                    return new
                todo.append(new)
                visit.add(new.loc)
    return

def traductionMouvement(input):
    mouvement = input%4
    indice_robot = input // 4
    return indice_robot, Direction(mouvement)

def mouvement(laby, input, liste_positions):
    indice_robot, direction = traductionMouvement(input)
    pos,liste = move_to_bidon(liste_positions[indice_robot], direction.value, laby, liste_positions)
    liste_positions[indice_robot] = liste[-1]
    return indice_robot, direction.name , liste

    



def main_bidon():
    nb_robot = 2
    ####### data robot #############
    # robotTankDirection = 0
    robotTankPos = (0,0)
    robotTwins1Pos = (3,0)
    robotTwins2Pos = (3,4)
    
    manager = Manager()
    robot_para = manager.dict()
    proc_list = []

    largeur = 5
    longueur = 3
    laby_size = (largeur,longueur)


    # the complete graph
    laby_complet = Labyrinthe(0,0,laby_size[0],laby_size[1], (0,0))
    laby_complet.init2DGraph()
    laby = laby_complet.graph
    node_not_explored = laby_complet.not_visited_node()

    # Divide node to visit

    allnodesDiviseBy3 = int(len(node_not_explored)/nb_robot)
    
    to_visit_r1 = node_not_explored[0:allnodesDiviseBy3]
    to_visit_r2 = node_not_explored[allnodesDiviseBy3:2*allnodesDiviseBy3]
    to_visit_r3 = node_not_explored[2*allnodesDiviseBy3:]

    
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
    # print (laby.nodes())

    #################### finish to complet laby ################
    print ("not finished explored" , to_visit)
    path_block = False
    # Path block should never be true
    
    while len(to_visit) > 0 and not path_block:
        print ("lists des positions des robots")
        print(listRobotPos)

        # find nearest node between 3 robots
        laby_complet.set_robot_pos(listRobotPos[0])
        path = None
        current_robot = 0
        for i in range(0, len(listRobotPos)):
            laby_complet.set_robot_pos(listRobotPos[i])
            new_path = laby_complet.nearest_node(to_visit)
            
            if (path == None or len(new_path) < len(path)) and len(new_path) > 0 :
                path = new_path
                current_robot = i
        


    #################### finish to complet laby ################
    print ("not finished explored" , not_visited)


    ################ SOLVER #####################
    robotTankPos = (0,1)
    robotTwins1Pos = (3,0)

    robot_list_pos = [robotTankPos, robotTwins1Pos]

    print ("fin operation liste des noeuds : ", to_visit)
    laby = laby_complet.graph
    ################ SOLVER #####################
    robotTankPos = (0,4)
    robotTwins1Pos = (0,0)
    robotTwins2Pos = (4,0)
    arrive = (2,2)

    robot_list_pos = [robotTankPos, robotTwins1Pos, robotTwins2Pos]
    pos = Pos(((robot_list_pos[2][0]*8+robot_list_pos[2][1])* 64 
    + robot_list_pos[1][0]*8+robot_list_pos[1][1])* 64 
    + robot_list_pos[0][0] * 8 + robot_list_pos[0][1], [])
    # pos.loc = (robot_list_pos[1][0]*8+robot_list_pos[1][1])* 64 + robot_list_pos[0][0] * 8 + robot_list_pos[0][1]

    robots_pos = solve_conf_bidon(laby, pos,
                arrive[0] * 8 + arrive[1])


    if robots_pos == None:
        print ("impossible")
    else: 
        print("position final des robots: ", robots_pos.loc)
        print ("liste des mouvements ")
        print (robots_pos.move)
        # print ([ traductionMouvement(k) for k in robots_pos.move])
        print ([ mouvement(laby, k, robot_list_pos) for k in robots_pos.move])
    print(traductionMouvement(1))
        
    ############### Drawing #####################
    pos = dict( (n, n) for n in laby.nodes() )
    nx.draw_networkx(laby, pos = pos) 
    nx.draw_networkx_nodes(laby,pos,
                       nodelist=[robotTankPos],
                       node_color='white')
    nx.draw_networkx_nodes(laby,pos,
                       nodelist=[robotTwins1Pos],
                       node_color='blue')
    nx.draw_networkx_nodes(laby,pos,
                       nodelist=[robotTwins2Pos],
                       node_color='purple')
    nx.draw_networkx_nodes(laby,pos,
                       nodelist=[arrive],
                       node_color='yellow')
    plt.axis('off')
    plt.show()

    
    # pos = dict( (n, n) for n in robot_para['laby_1'].nodes() )
    # nx.draw_networkx(robot_para['laby_1'], pos = pos) 
    # plt.axis('off')
    # plt.show()


if __name__ == '__main__':
    # main()
    # main_bidon()
    robotPosition = [(0,0),(2,4),(7,0)]
    arrive = (4,4)
    server = Server('192.168.1.47',3,5,robotPosition, 2, True)
    # server.connectA()
    # server.connectB()
    # server.connectC()
    server.run()
    # server.solve([(0,4), (0,0), (4,0)], arrive)
