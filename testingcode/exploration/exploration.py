#!/usr/bin/env python3
import networkx as nx
from networkx.drawing import nx_agraph
import re
import matplotlib.pyplot as plt
# first exploration algorithm (just one robot in the maze)
# return the graph of the maze (sould be done in less than 5 minutes which mean 4.68 sec by square)
def basicExploration(robotExplorer):
    # creer graphe vide
    # faire tourner le robt sur lui meme 
    return


def initGraph():
    graph = nx.grid_2d_graph(8,8)
    return graph

def displayGraph(graph):
    print(graph.edges())
    pos = dict( (n, n) for n in graph.nodes() )
    labels = dict( ((i, j), i + (8-1-j) * 10 ) for i, j in graph.nodes() )
    nx.draw_networkx(graph, pos=pos, labels=labels)
    plt.axis('off')
    plt.show()

def main():
    graph = initGraph()
    displayGraph(graph)    

if __name__ == '__main__':
    main()


