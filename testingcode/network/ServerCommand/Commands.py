#!/usr/bin/env python3
import sys
from enum import Enum
class dataID(Enum):
    commandFail = -1
    commandSuccess = 0
    commandSuccessWithData = 1
    
class ServerCommand: 
    _pos = None
    _dataId = None
    _stateWalls = None
    def __init__(self, id):
        self._dataId = id
    def __str__(self):
        return "Server Command : ID = " + str(self._dataId) + " \n"
    def setPos(self,pos) :
        self._pos = pos
    def setWallsState(self,state) :
        self._stateWalls = state
    def setId(self,id) :
        self._dataId = s
    def doCommand(self):
        print("Commande vide", file=sys.stderr)

