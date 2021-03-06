#!/usr/bin/env python3
import networkx as nx
from networkx.drawing import nx_agraph
import re, os, sys
import matplotlib.pyplot as plt

sys.path.append(os.path.realpath('./'))
from testingcode.exploration import robot
from enum import Enum
from testingcode.exploration import directionEnum

import random
import copy

class Color(Enum) :
    red = 0
    purple = 1
    blue = 2

class Labyrinthe:
    dim_x = None
    dim_y = None
    robot_pos = None
    graph = None

    # TODO: check avec oreste pour sur la pertinence de la class robot
    # def __init__(self, dim_x, dim_y, robot, robot_pos):
    #     self.dim_x = dim_x
    #     self.dim_y = dim_y
    #     self.robot = robot
    #     self.robot_pos = robot_pos # a tuple (i,j)

    def __init__(self, x, y, dim_x, dim_y, robot_pos):
        self.offset_x = x
        self.offset_y = y
        self.dim_x = dim_x
        self.dim_y = dim_y
        self.robot_pos = robot_pos # a tuple (i,j)
        self.graph = None
        self.cut_graph = None
    
    def init_Labyrinthe(self):
        graph = nx.grid_2d_graph(self.dim_x,self.dim_y)
        self.graph = graph
        nx.set_node_attributes(self.graph, 'etat', 0)
        
        #print(self.graph.nodes(True))
        return graph

    def init_Pos_Robot(self):
        attrs = {(self.robot.pos_x,self.robot.pos_y): {'etat':1}}
        print(attrs)
        nx.set_node_attributes(self.graph,attrs)
        return
        
    def set_robot_pos(self, pos):
        self.robot_pos = pos

    def display_Graph(self):
        pos = dict( (n, n) for n in self.graph.nodes() )
        labels = dict( ((i, j), (i,j) ) for i, j in self.graph.nodes() )
        print(labels)
        etats=nx.get_node_attributes(self.graph,'etat')
        colors = list( (Color(etats[n]).name) for n in self.graph.nodes() )
        #c = dict( (n, Color(etats[n]).name) for n in self.graph.nodes() )
        # print(c)
        
        nx.draw_networkx(self.graph, pos=pos, labels=labels, font_size=8, node_size=600, node_color=colors) 
        plt.axis('off')
        plt.show()

    '''
    return 1 case on 2 of a grid
    Not dynamics, start case.

    :return: list of tupple
    '''
    def not_visited_node(self):
        nodes = []

        for i in range (self.offset_x, self.offset_x + self.dim_x):
            for j in range (self.offset_y + (i%2), self.offset_y + self.dim_y, 2):
                nodes.append((i,j))
        
        return nodes
    
    '''
        graph must be define
    '''
    def init_cut_graph(self,node1, node2):
        x1, y1 = node1
        s = x1 * (self.offset_y + self.dim_y) + y1
        x2, y2 = node2
        e = x2 * (self.offset_y + self.dim_y) + y2
        self.cut_graph = copy.deepcopy(self.graph)
        
        for i in range (self.offset_x, self.offset_x + self.dim_x):
            for j in range (self.offset_y, self.offset_y + self.dim_y):
                v = i * (self.offset_y + self.dim_y) + j
                if v < s or v > e:
                    self.cut_graph.remove_node((i, j))

        return self.cut_graph
    
    def nearest_node(self, target_nodes, on_cut_laby = False):
        if on_cut_laby:
            laby = self.cut_graph
        else:
            laby = self.graph

        short_path = []
        short_path_lg = self.dim_x * self.dim_y
        for target in target_nodes:
            if nx.has_path(laby, self.robot_pos, target):
                path = nx.shortest_path(laby, self.robot_pos, target)

                if len(path) < short_path_lg:
                    short_path_lg = len(path)
                    short_path = path
        
        return short_path

    def init2DGraph(self):
        g = nx.Graph()
        
        # fill line edges
        for i in range (self.offset_x, self.offset_x + self.dim_x):
            for j in range(self.offset_y, self.offset_y + self.dim_y -1):
                g.add_edge( (i,j), (i,j+1))
        
        # fill column edges
        for i in range (self.offset_x, self.offset_x + self.dim_x - 1):
            for j in range(self.offset_y, self.offset_y + self.dim_y):
                g.add_edge( (i,j), (i+1,j))

        self.graph = g
        return g

    def dot_to_nxGraph(self, path):
        """
            read a graph dot file then return a nxGraph

            vertex is a string
            vertex is compose of 2 int (a pair)
            return a graph of pair of int
        """
    
        Gstr = nx_agraph.read_dot(path)
        Gpair = nx.Graph()

        alledges = list(Gstr.edges())
        expr = '(\d+)'

        # create a graph of pair
        for node1, node2 in alledges:
            # extract i,j of vertex
            tmp = re.findall(expr, node1)
            i1 = int(tmp[0])
            j1 = int(tmp[1])

            tmp = re.findall(expr, node2)
            i2 = int(tmp[0])
            j2 = int(tmp[1])

            Gpair.add_edge((i1, j1), (i2, j2))

        self.graph = Gpair
        return Gpair

    def direction(self, nodeDepart, nodeDestination):
        d = directionEnum.Direction
        if nodeDestination[0] - nodeDepart[0] > 0:
            val = directionEnum.Direction.DOWN
        elif nodeDestination[0] - nodeDepart[0] < 0:
            val = directionEnum.Direction.UP
        elif nodeDestination[1] - nodeDepart[1] > 0:
            val = directionEnum.Direction.RIGHT
        elif nodeDestination[1] - nodeDepart[1] < 0:
            val = directionEnum.Direction.LEFT
        else:
            val = None
        return val
    def path2Command(self,path):
        #print("Converting path to command \n path is ", path)
        command = []
        for i in range(len(path) - 1) :
            direction = self.direction(path[i],path[i+1]).value
            command.append((direction,1))
        #print("Converted path is ", command)
        return command

    def edge_by_direction(self, pos, direction):
        if direction == 2:
            next_pos = (pos[0], pos[1] -1)
        elif direction == 1:
            next_pos = (pos[0] -1 , pos[1])
        elif direction == 0:
            next_pos = (pos[0], pos[1] + 1)
        elif direction == 3:
            next_pos = (pos[0] + 1, pos[1])
        else:
            print("direction not recognized")

        return (pos, next_pos)

    def get_robot_pos(self):
        return self.robot_pos

    def adjacents(self, node):
        return self.graph.neighbors(node)

    def check_adjacents_all_visited(self,node):
        for adj in self.adjacents(node):
            etat = self.graph.node[adj]['etat']
            if (not(etat == 1 or etat == 2)):
                return
        x, y = node                                 #ou
        attrs = {(x, y): {'etat':2}}                #self.graph.node[pos]['etat'] = 2
        nx.set_node_attributes(self.graph,attrs)    #?

    def deplacer_explo_robot(self, nodeDestination):
        etat = self.graph.node[nodeDestination]['etat']
        if etat == 0:
            self.graph.node[nodeDestination]['etat'] = 1
        check_adjacents_all_visited(get_robot_pos)
        self.set_robot_pos(nodeDestination)
        for adj in self.adjacents(nodeDestination):
            check_adjacents_all_visited(adj)

    # A qui est ce code? dire à bao anh
    # def exploration(self):
    #     while(path = self.nearest_node(self.not_visited_node())):
    #         for node in path:
    #             self.deplacer_explo_robot(node)
    
    def set_size(self, x, y, dim_x, dim_y):
        self.offset_x = x
        self.offset_y = y
        self.dim_x = dim_x
        self.dim_y = dim_y


def main():
    pass
    # robot = Robot(4,4,1)
    # c = Color(1).name
    # print(c)
    # labyrinthe = Labyrinthe(4,4,robot, (0,1))
    # labyrinthe.init_Labyrinthe()
    # labyrinthe.init_Pos_Robot()
    # labyrinthe.display_Graph()
        

    robotPos = (0,0)
    laby = Labyrinthe(0, 0, 2,2,robotPos)
    laby.init2DGraph()

    # TODO il ne faut pas géré le cas départ ou le robot est sur le noeud avec un remove
    # TODO certe ça optimise mais pour la généricité quand le robot doit aider les autres c'est mieux.
    # calculate all shortest path.
    to_visit = laby.not_visited_node()
    # to_visit.remove(robotPos)
    path_block = False


    path_block = False
    
    edge_deleted = []

    print (laby.graph)
    while len(to_visit) > 0 and not path_block:
        # print ("to_visit = ", to_visit)
        path = laby.nearest_node(to_visit)
        
        # print("path = ", path)

        # if there are a path (we also consider case where robot is already on correct case for generic)
        if len(path) > 0:
            # simulation of block during go to node
            if (len(path) > 1):
                success = random.choice([True,False])
                success = False
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
    
    pos = dict( (n, n) for n in laby.graph.nodes() )
    nx.draw_networkx(laby.graph, pos = pos) 
    plt.axis('off')
    plt.show()

if __name__ == '__main__':
    main()