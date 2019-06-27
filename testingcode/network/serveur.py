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
import matplotlib.animation


from command import Command
from RobotCommand.robotCommand import RobotCommand
from RobotCommand.moveForward import RobotMoveForward
from RobotCommand.turn180 import RobotTurn180
from RobotCommand.turnLeft import RobotTurnLeft
from RobotCommand.turnRight import RobotTurnRight

# sys.path.insert(0, "D:\\Data\\Travail\\Universite\\master\\idefix")
sys.path.append("/home/etu/p1406781/Informatique/M1/IDEFIX/idefix")
from testingcode.exploration.labyrinthe import Labyrinthe
from testingcode.exploration.directionEnum import Direction
import testingcode.solver.solve
from testingcode.solver.maze import Pos
import copy


class Server:
    def __init__(self, dim_x, dim_y):
        robot_0_pos = (0,0)
        robot_1_pos = (2,0)
        robot_2_pos = (3,0)

        self.graph_dim = (dim_x,dim_y)
        self.labyrinth = Labyrinthe(0,0,dim_x,dim_y, (0,0)) # position robot not
        self.labyrinth.init2DGraph()
    

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
            path = laby.nearest_node(to_visit)
            # if there are a path (we also consider case where robot is already on correct case for generic)
            if len(path) > 0:
            
                success,laby, temp = self.try_to_go(path, laby)
                edges_deleted += temp
                if success:
                    to_visit.remove(path[-1])
            else:
                path_block = True
        
        output_para['to_visit_' + str(id)] = to_visit
        output_para['laby_' + str(id)] = laby.graph
        output_para['deleted_edge_' + str(id)] = edges_deleted


    '''
        make robot try to follow a path
        :return: updated laby
    '''
    def try_to_go(self, path, laby):
        success = None
        edges_deleted = []

        
        ###### simulation of block during go to node
        if (len(path) > 1):
            success = random.choice([True,False, True, True])
            if not success:
                node_block = random.randint(1, len(path) - 1)
        else:
            success = True
        #### end of simulation ######
        # success, edge_block = robot.robot_follow(path)
        #### real code up #########

        ### robot successfully achieved path
        if (success):
            laby.set_robot_pos(path[-1])
            
            #### simumation of scaning
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
            #### end of simulation
            # edges = robot.robot_scan() # socket scan
            ####### real code up #######

            laby.graph.remove_edges_from(edges_deleted)
            laby.cut_graph.remove_edges_from(edges_deleted)
            to_visit.remove((path[-1]))
        
        ### robot get blocked during moving
        else:
            ###### simulation of block during go to node ####
            laby.set_robot_pos(path[node_block-1])
            laby.graph.remove_edge(path[node_block-1],  path[node_block])
            edges_deleted.append((path[node_block-1],  path[node_block]))
            ##### end of simulation
            # laby.set_robot_pos(edge_block[0]) # socket get position
            # laby.graph.remove(edge_block) # socket get edge deleted
            ####### real code up #######



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
        
        allnodesDiviseBy3 = int(len(node_not_explored)/3)
        to_visit_list = []
        for i in range (0, 3):
            s = i*allnodesDiviseBy3
            if i+1 > 2:
                e = len(node_not_explored)
            else:
                e = (i+1)*allnodesDiviseBy3
            to_visit_list.append(node_not_explored[s:e])
            laby_to_explore_list[i].init_cut_graph(node_not_explored[s],node_not_explored[e-1])


        ############### NOT finish to code ###########

        # i = 0
        # for robot in self.robot_list:
        #     proc_list.append(Process(target=robot.correct_labyrinth, args=(i, to_vist)))
        #     proc_list[i].start()
        #     i += 1

        # not_visited = []
        # for i in range (0, len(self.robot_list)):
        #     proc_list[i].join()
        #     laby = nx.union(laby, self.robot_list[i].get_labyrinthe())
        #     not_visited.append(to_vist[i])
        

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
        
        # self.labyrinth = laby

    def move_to(self, robot_p, direction):
        pos = robot_p.get_position()
        can_go = True
        if direction ==  Direction.LEFT:
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
        if direction == 2: #Direction.LEFT
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
        elif direction == 1: #Direction.UP
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
        elif direction == 0: #Direction.RIGHT
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
        elif direction == 3: #Direction.DOWN
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
    list_mvt = [pos]    
    can_go = True
    if direction == 3:
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
    elif direction == 2:
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
    elif direction == 1:
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

    elif direction == 0:
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
                # success = random.choice([True,False, True, True])
                success = True
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

    
def animation(laby, robot_list_pos, arrive, mouvements):
    fig, ax = plt.subplots(figsize=(10,10))
    pos = dict( (n, n) for n in laby.nodes() )
    sequence = [(0,0),(0,1),(0,2),(1,0),(1,1),(1,2)]
    ani = matplotlib.animation.FuncAnimation(fig, update, frames=6, interval=1000, repeat=True, fargs=(laby, arrive, pos, ax, robot_list_pos, mouvements))
    plt.show()

def update(num, laby, arrive, pos, ax, robot_list_pos, mouvements):
    ax.clear()
    if len(mouvements) > 0 :
        if len(mouvements[0][2]) > 0 :
            indice_robot, position = mouvements[0][0], mouvements[0][2].pop(0)
            robot_list_pos[indice_robot] = position
        else :
            mouvements.pop(0)
            if len(mouvements) > 0 :
                indice_robot, position = mouvements[0][0], mouvements[0][2].pop(0)
                robot_list_pos[indice_robot] = position
        

    # nx.draw_networkx(laby, pos = pos)
    i = num // 3
    path = robot_list_pos
    
    # Background nodes
    nx.draw_networkx_edges(laby, pos=pos, ax=ax, edge_color="gray")
    null_nodes = nx.draw_networkx_nodes(laby, pos=pos, nodelist=set(laby.nodes()) - set(path) - set(arrive), node_color="white",  ax=ax)
    null_nodes.set_edgecolor("black")

    # Query nodes
    query_nodes = nx.draw_networkx_nodes(laby, pos=pos, nodelist=[arrive], node_color="yellow", ax=ax)
    query_nodes = nx.draw_networkx_nodes(laby, pos=pos, nodelist=[path[0]], node_color="red", ax=ax)
    query_nodes = nx.draw_networkx_nodes(laby, pos=pos, nodelist=[path[1]], node_color="green", ax=ax)
    query_nodes = nx.draw_networkx_nodes(laby, pos=pos, nodelist=[path[2]], node_color="blue", ax=ax)
    

    # query_nodes = nx.draw_networkx_nodes(laby, pos=pos, nodelist=path, node_color="blue", ax=ax)
    query_nodes.set_edgecolor("purple")
    # nx.draw_networkx_labels(laby, pos=pos, labels=dict(zip(path,path)),  font_color="blue", ax=ax)
    # edgelist = [path[k:k+2] for k in range(len(path) - 1)]
    # nx.draw_networkx_edges(laby, pos=pos, edgelist=edgelist, ax=ax)

    # Scale plot ax
    ax.set_title("Frame %d:    "%(num+1) + str(path), fontweight="bold")
    ax.set_xticks([])
    ax.set_yticks([])




def main_bidon():
    ####### data robot #############
    # robotTankDirection = 0
    robotTankPos = (0,0)
    robotTwins1Pos = (3,0)
    robotTwins2Pos = (4,0)

    listRobotPos = [robotTankPos, robotTwins1Pos, robotTwins2Pos]
    
    manager = Manager()
    robot_para = manager.dict()
    proc_list = []
    largeur = 8
    longueur = 8
    laby_size = (largeur,longueur)


    # the complete graph
    laby_complet = Labyrinthe(0,0,laby_size[0],laby_size[1], (0,0))
    laby_complet.init2DGraph()
    laby = laby_complet.graph
    node_not_explored = laby_complet.not_visited_node()

    # Divide node to visit
    allnodesDiviseBy3 = int(len(node_not_explored)/3)
    
    to_visit_r1 = node_not_explored[0:allnodesDiviseBy3]
    to_visit_r2 = node_not_explored[allnodesDiviseBy3:2*allnodesDiviseBy3]
    to_visit_r3 = node_not_explored[2*allnodesDiviseBy3:]

    
    proc_list.append(Process(target=correct_labyrinth_bidon, args=(0, robot_para, robotTankPos, laby_complet, to_visit_r1)))
    proc_list[0].start()
    proc_list.append(Process(target=correct_labyrinth_bidon, args=(1, robot_para, robotTwins1Pos, laby_complet, to_visit_r2)))
    proc_list[1].start()
    proc_list.append(Process(target=correct_labyrinth_bidon, args=(2, robot_para, robotTwins2Pos, laby_complet, to_visit_r3)))
    proc_list[2].start()

    # exploration data of robot
    to_visit = []
    deleted_edge = []
    for i in range (0, 3):
        proc_list[i].join()
        to_visit += robot_para['to_visit_' + str(i)]
        deleted_edge += robot_para['deleted_edge_' + str(i)]
    

    # for i in range()
    # laby = nx.union(laby_1.graph, laby_2.graph)
    # laby = nx.union(robot_para['laby_0'], robot_para['laby_1'])
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
        
        print ("le chemin a faire du robot : ", current_robot, " a faire :")
        print (path)


        # if there are a path (we also consider case where robot is already on correct case for generic)
        if path is not None and len(path) > 0:
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
                laby_complet.graph.remove_edges_from(edges)
                to_visit.remove((path[-1]))
                laby_complet.set_robot_pos(path[-1])
            
            else:
                robotPos = (path[node_block-1])
                laby_complet.set_robot_pos(path[node_block-1])
                laby_complet.graph.remove_edge(path[node_block-1],  path[node_block])
            
            # update current robot pos
            listRobotPos[current_robot] = robotPos
                
        else:
            # print ("PATH BLOCK")
            path_block = True

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

    mouvements = []

    if robots_pos == None:
        print ("impossible")
    else: 
        print("position final des robots: ", robots_pos.loc)
        print ("liste des mouvements ")
        print (robots_pos.move)
        # print ([ traductionMouvement(k) for k in robots_pos.move])
        mouvements = [ mouvement(laby, k, robot_list_pos) for k in robots_pos.move]
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
    # plt.axis('off')
    # plt.show()
    animation(laby, [robotTankPos, robotTwins1Pos, robotTwins2Pos], arrive, mouvements)


if __name__ == '__main__':
    # main()
    main_bidon()

    # server = Server(6,6)
    # server.run()