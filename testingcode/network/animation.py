import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation

import sys
sys.path.append("/home/etu/p1406781/Informatique/M1/IDEFIX/idefix")
from testingcode.exploration.labyrinthe import Labyrinthe
from testingcode.exploration.directionEnum import Direction
import testingcode.solver.solve
from testingcode.solver.maze import Pos
import copy

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
            
            pos_r0 = move_to_bidon(pos_r0, i%4, lab, [pos_r0, pos_r1, pos_r2])

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

def move_to_bidon(robot_pos, direction, laby, robot_list_pos):
    pos = robot_pos
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

    else:
        print("direction not recognized")

    return pos


# Create Graph
np.random.seed(2)
G = nx.cubical_graph()
G = nx.relabel_nodes(G, {0:"O", 1:"X", 2:"XZ", 3:"Z", 4:"Y", 5:"YZ", 6: "XYZ", 7:"XY"})
pos = nx.spring_layout(G)

laby_size = (4,4)
laby_complet = Labyrinthe(0,0,laby_size[0],laby_size[1], (0,0))
laby_complet.init2DGraph()
laby = laby_complet.graph

robotTankPos = (0,0)
robotTwins1Pos = (0,0)
robotTwins2Pos = (0,0)
arrive = (2,2)

robot_list_pos = [robotTankPos, robotTwins1Pos, robotTwins2Pos]
# pos = Pos(((robot_list_pos[2][0]*8+robot_list_pos[2][1])* 64 
# + robot_list_pos[1][0]*8+robot_list_pos[1][1])* 64 
# + robot_list_pos[0][0] * 8 + robot_list_pos[0][1], [])
# pos.loc = (robot_list_pos[1][0]*8+robot_list_pos[1][1])* 64 + robot_list_pos[0][0] * 8 + robot_list_pos[0][1]

#robots_pos = solve_conf_bidon(laby, pos, arrive[0] * 8 + arrive[1])


# if robots_pos == None:
#     print ("impossible")
# else: 
#     print("position final des robots: ", robots_pos.loc)
#     print ("liste des mouvements ")
#     print (robots_pos.move)
    
############### Drawing #####################
# pos = dict( (n, n) for n in laby.nodes() )
# nx.draw_networkx(laby, pos = pos) 
# nx.draw_networkx_nodes(laby,pos,
#                     nodelist=[robotTankPos],
#                     node_color='white')
# nx.draw_networkx_nodes(laby,pos,
#                     nodelist=[robotTwins1Pos],
#                     node_color='blue')
# nx.draw_networkx_nodes(laby,pos,
#                     nodelist=[robotTwins2Pos],
#                     node_color='purple')
# nx.draw_networkx_nodes(laby,pos,
#                     nodelist=[arrive],
#                     node_color='yellow')
# plt.axis('off')
# plt.show()

# Sequence of letters
sequence_of_letters = "".join(['X', 'Y', 'Z', 'Y', 'Y', 'Z'])
# sequence_of_letters = [(0,0),(0,1)]
idx_weights = [3,2,1]

# Build plot
fig, ax = plt.subplots(figsize=(5,5))
fig2, ax2 = plt.subplots(figsize=(6,4))


def update(num):
    ax.clear()
    i = num // 3
    j = num % 3 + 1
    triad = sequence_of_letters
    path = ["O"] + ["".join(sorted(set(triad[:k + 1]))) for k in range(j)]

    # Background nodes
    nx.draw_networkx_edges(G, pos=pos, ax=ax, edge_color="gray")
    null_nodes = nx.draw_networkx_nodes(G, pos=pos, nodelist=set(G.nodes()) - set(path), node_color="white",  ax=ax)
    null_nodes.set_edgecolor("black")

    # Query nodes
    query_nodes = nx.draw_networkx_nodes(G, pos=pos, nodelist=path, node_color="grey", ax=ax)
    query_nodes.set_edgecolor("white")
    nx.draw_networkx_labels(G, pos=pos, labels=dict(zip(path,path)),  font_color="white", ax=ax)
    edgelist = [path[k:k+2] for k in range(len(path) - 1)]
    nx.draw_networkx_edges(G, pos=pos, edgelist=edgelist, width=idx_weights[:len(path)], ax=ax)

    # Scale plot ax
    ax.set_title("Frame %d:    "%(num+1) +  " - ".join(path), fontweight="bold")
    ax.set_xticks([])
    ax.set_yticks([])

def update2(num):
    ax2.clear()
    triad = sequence_of_letters[num%2]
    path = [(0,0)] + [triad]; 

    # Background nodes
    nx.draw_networkx_edges(laby_complet, pos=pos, ax=ax2, edge_color="gray")
    null_nodes = nx.draw_networkx_nodes(laby_complet, pos=pos, nodelist=set(G.nodes()) - set(path), node_color="white",  ax=ax)
    null_nodes.set_edgecolor("black")

    # Query nodes
    query_nodes = nx.draw_networkx_nodes(laby_complet, pos=pos, nodelist=path, node_color="grey", ax=ax2)
    query_nodes.set_edgecolor("white")
    nx.draw_networkx_labels(laby_complet, pos=pos, labels=dict(zip(path,path)),  font_color="white", ax=ax2)
    edgelist = [path[k:k+2] for k in range(len(path) - 1)]
    nx.draw_networkx_edges(laby_complet, pos=pos, edgelist=edgelist, width=idx_weights[:len(path)], ax=ax2)

    # Scale plot ax
    ax2.set_title("Frame %d:    "%(num+1) +  " - ".join(path), fontweight="bold")
    ax2.set_xticks([])
    ax2.set_yticks([])



ani = matplotlib.animation.FuncAnimation(fig, update, frames=6, interval=1000, repeat=True)
# ani = matplotlib.animation.FuncAnimation(fig2, update2, frames=6, interval=1000, repeat=True)
plt.show()