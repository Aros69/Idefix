#!/usr/bin/env python3
import sys
from enum import Enum
class dataID(Enum):
    commandFail = -1
    commandSuccess = 0
    
class ServerCommand: 
    _pos = None
    _dataId = None
    def __init__(self, id):
        self._dataId = id
    def setPos(self,pos) :
        self._pos = pos
    def setId(self,id) :
        self._dataId = s
    def doCommand(self):
        print("Commande vide", file=sys.stderr)

