import time

class Chrono:
    # DATA
    _startTime = 0
    _endTime = 0

    # METHODS
    def __init__(self):
        self._startTime = 0
        self._endTime = 0

    def start(self):
        self._startTime = time.time()
        return self._startTime

    def stop(self):
        self._endTime = time.time()
        res = self._endTime - self._startTime 
        #self.reset()
        return res

    def reset(self):
        self._startTime = 0
        self._endTime = 0
    
    def getChrono(self):
        return self._endTime - self._startTime

def main():
    c = Chrono()
    c.start()
    time.sleep(5)
    r = c.stop()
    print(r)

if __name__ == '__main__':
    main()