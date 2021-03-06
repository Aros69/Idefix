#!/usr/bin/env python3

import time, os, sys

class Chrono:
    # DATA
    _startTime = 0
    _endTime = 0
    _lapsTime = 0

    # METHODS
    def __init__(self):
        self._startTime = 0
        self._endTime = 0
        self._lapsTime = 0

    def start(self):
        self._startTime = time.time()
        return self._startTime

    def getTime(self):
        return self._lapsTime + (time.time() - self._startTime)

    def pause(self):
        self._lapsTime += time.time() - self._startTime 

    def restart(self):
        self._startTime = time.time()

    def stop(self):
        self._endTime = time.time()
        res = self._lapsTime + (self._endTime - self._startTime)
        return res

    def reset(self):
        self._startTime = 0
        self._endTime = 0
        self._lapsTime = 0
    
    def getChrono(self):
        return self._endTime - self._startTime

def main():
    c = Chrono()
    c.start()
    time.sleep(5)
    r = c.getTime()
    print(r)

if __name__ == '__main__':
    main()
