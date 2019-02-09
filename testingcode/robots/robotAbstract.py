#!/usr/bin/env python3
from abc import ABCMeta, abstractmethod

# NOT USED : only educational purpose

# Abstract class for robots
# abc is the package to create an abstract class
class RobotAbstract(metaclass=ABCMeta):

    @abstractmethod # defining an abstract method
    def __init__(self):
        pass # if a code is written, the method MUST BE implemented in inherited class

    @abstractmethod
    def turnLeft(self):
        pass

    @abstractmethod
    def turnRight(self):
        pass
    
    @abstractmethod
    def turn180(self):
        pass
    
    @abstractmethod
    def moveForward(self):
        pass
    
    @abstractmethod
    def bothMotorsRotation(self):
        pass

    @abstractmethod
    def leftMotorRotation(self):
        pass

    @abstractmethod
    def rightMotorRotation(self):
        pass
 
    @abstractmethod
    def stopMotor(self):
        pass
    
    @abstractmethod
    def activateColorSensor(self):
        pass

# registering the robots class as inherited class
#RobotAbstract.register(...)
