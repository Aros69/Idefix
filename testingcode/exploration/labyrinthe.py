#!/usr/bin/env python3
import networkx as nx
from networkx.drawing import nx_agraph
import re
import matplotlib.pyplot as plt
from testingcode.exploration import robot
from enum import Enum
from testingcode.exploration import directionEnum

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

    def __init__(self, dim_x, dim_y, robot_pos):
        self.dim_x = dim_x
        self.dim_y = dim_y
        self.robot_pos = robot_pos # a tuple (i,j)
    
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

    :return: list of tupple
    '''
    def not_visited_node(self):
        node = []
        j = 0
        for i in range (0, self.dim_x):
            # if pair number
            for j in range (j, self.dim_y, 2):
                node.append((i,j))

            j = j% self.dim_y-1
            j = j%2
        return node
    
    def nearest_node(self, target_nodes):
        short_path = []
        short_path_lg = self.dim_x * self.dim_y
        for target in target_nodes:
            if nx.has_path(self.graph, self.robot_pos, target):
                path = nx.shortest_path(self.graph, self.robot_pos, target)

                if len(path) < short_path_lg:
                    short_path_lg = len(path)
                    short_part = path
        
        return short_part

    def init2DGraph(self):
        g = nx.Graph()

        for i in range (0, self.dim_x):
            for j in range (0, self.dim_y):
                x = i
                y = j

                if i > 0 and i < (self.dim_x-1):
                    g.add_edge( (i,j),(i-1,j) )
                    g.add_edge( (i,j),(i+1,j) )
                if j > 0 and j < (self.dim_y-1):
                    g.add_edge( (i,j),(i,j-1) )
                    g.add_edge( (i,j),(i,j+1) )

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
        Gpair = nx.DiGraph()

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
    

def main():
    pass
    # robot = Robot(4,4,1)
    # c = Color(1).name
    # print(c)
    # labyrinthe = Labyrinthe(4,4,robot, (0,1))
    # labyrinthe.init_Labyrinthe()
    # labyrinthe.init_Pos_Robot()
    # labyrinthe.display_Graph()
   
    
    # calculate all shortest path.
    # to_visit = labyrinthe.not_visited_node()
    # path = labyrinthe.nearest_node(to_visit)
    # transforme into robot direction mouvement
    # if chemin don't exist.
    # update graph then loop
    # graph.remove_edge
    
    # if destinations.
    # scan, update graph
    # remove node from not visited, then check for next shortest path.

if __name__ == '__main__':
    main()