#!/usr/bin/env python3
import networkx as nx
from networkx.drawing import nx_agraph
import re
import matplotlib.pyplot as plt
from robot import Robot
from enum import Enum

class Color(Enum) :
    red = 0
    purple = 1
    blue = 2

class Labyrinthe:
    def __init__(self, dim_x, dim_y, robot):
        self.dim_x = dim_x
        self.dim_y = dim_y
        self.robot = robot

    def init_Labyrinthe(self):
        graph = nx.grid_2d_graph(self.dim_x,self.dim_y)
        self.graph = graph;
        nx.set_node_attributes(self.graph, 'etat', 0)
        
        #print(self.graph.nodes(True))
        return graph

    def init_Pos_Robot(self):
        #etats=nx.get_node_attributes(self.graph,{(robot.pos_x,)})
        return
        

    def display_Graph(self):
        pos = dict( (n, n) for n in self.graph.nodes() )
        labels = dict( ((i, j), (i,j) ) for i, j in self.graph.nodes() )
        print(labels)
        etats=nx.get_node_attributes(self.graph,'etat')
        colors = list( (Color(etats[n]).name) for n in self.graph.nodes() )
        print(colors)
        
        nx.draw_networkx(self.graph, pos=pos, labels=labels, font_size=8, node_size=600, node_color=colors) 
        plt.axis('off')
        plt.show()

def main():
    robot = Robot(4,4,1)
    c = Color(1).name
    print(c)
    labyrinthe = Labyrinthe(8,8,robot)
    labyrinthe.init_Labyrinthe()
    labyrinthe.init_Pos_Robot()
    labyrinthe.display_Graph()

if __name__ == '__main__':
    main()