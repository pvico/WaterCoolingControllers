#########################################################
#                                                       #
#                       PID.py                          #
#                   PID Controller class                #
#                   Philippe Vico 2016                  #
#                                                       #
#########################################################

# Time reference implemented with time.time() which in
# MicroPython is an integer equals to the number of seconds
# since 1/1/2000. This is ok for slow changing processes
# like temperature

import utime

class PID:
    def __init__(self, setValue=0.0, kP=0.0, kI=0.0, kD=0.0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.errorIntegration = 0.0
        self.correctionProportional = 0.0
        self.correctionIntegral = 0.0
        self.correctionDerivative = 0.0
        self.setValue = setValue
        self.output = 0.0
        self.lastError = 0.0
        self.lastTime= utime.time()      # number of seconds since 1/1/2000 as an integer

    @micropython.native
    def update(self, sensedValue):
        now = utime.time()
        error = sensedValue - self.setValue
        dt = 1.0 * (now - self.lastTime)
        de = 1.0 * (error - self.lastError)
        self.correctionProportional = self.kP * error
        self.errorIntegration = max((-50.0/self.kI), self.errorIntegration + (error * dt))
        self.correctionIntegral = self.errorIntegration * self.kI
        self.correctionDerivative = (self.kD * de/dt) if dt > 0.0 else 0.0
        self.lastTime = now
        self.lastError = error
        self.output = int(min(100, max(0, 50 + self.correctionProportional + self.correctionIntegral + self.correctionDerivative)))
        return self.output

    def printState(self):
        print("kP: %f" % self.kP)
        print("kI: %f" % self.kI)
        print("kD: %f" % self.kD)
        print("errorIntegration: %f" % self.errorIntegration)
        print("correctionProportional: %f" % self.correctionProportional)
        print("correctionIntegral: %f" % self.correctionIntegral)
        print("correctionDerivative: %f" % self.correctionDerivative)
        print("setValue: %f" % self.setValue)
        print("output: %d" % self.output)
        print("lastError: %f" % self.lastError)
        print("lastTime: %d" % self.lastTime)
