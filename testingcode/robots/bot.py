import os
import sys
import time
import random

class bot :
    def __init__(self) :
        print("Bot init",sys.stderr)

    def goThere(self,newDirection, step):
        print ("Generic Robot Movement in ",newDirection," of ", step , " units ",sys.stderr)

    def executeCommands(self,commands) :
        print ("Generic Commands list Execution")
    