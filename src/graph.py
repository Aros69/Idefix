# from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_C, OUTPUT_B,SpeedRPM, SpeedPercent, MoveTank
# from ev3dev2.sensor import INPUT_1
# from ev3dev2.sensor.lego import TouchSensor
# from ev3dev2.led import Leds
import pygraphviz as pgv
import networkx as nx
from networkx.drawing import nx_agraph
import re
import directionEnum


def dot_to_nxGraph(path):
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

def direction(nodeDepart, nodeDestination):
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

if __name__ == '__main__':
    print("yo man")
    
    ##################### Creating graph #################

    # Creating graph from dot file
    # G = dot_to_nxGraph("inputLaby.dot")
    
    # Creating manually graph
    G = nx.Graph()

    ###################### manipulating graph ###############
    
    G.add_node((0,0))
    G.add_node((1,1))
    G.add_edge((0,0), (1,2))
    G.add_edge((1,0), (1,2))

    alledges = list(G.edges())
    neighbors = list(G.neighbors((0,0)))
    allnodes = list(G.nodes())


    ##################### Drawing/writing graph using pygraphviz #####################
    G_pgv = nx_agraph.to_agraph(G)

    # ATTENTION relative path.
    G_pgv.write('./labyrinthe.dot') #Dot file
    G_pgv.draw('./labyrinthe.png', format='png', prog='dot') #png file

    print("yay sucess")
