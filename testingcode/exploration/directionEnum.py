from enum import Enum, unique

@unique
class Direction(Enum):
    LEFT = 0
    RIGHT = 1
    DOWN = 2
    UP = 3
