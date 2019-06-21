#!/usr/bin/env python3
import networkx as nx
from networkx.drawing import nx_agraph
import re
import matplotlib.pyplot as plt
from enum import Enum

class Direction(Enum) :
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3

class Robot:
    def __init__(self, pos_x, pos_y, direction):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.direction = Direction(direction)