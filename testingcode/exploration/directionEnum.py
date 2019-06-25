from enum import Enum, unique

@unique
class Direction(Enum):
    LEFT = 0
    RIGHT = 2
    DOWN = 3
    UP = 1
