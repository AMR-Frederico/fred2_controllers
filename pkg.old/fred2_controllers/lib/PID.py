#!/user/bin/env python3

from time import time 

class PID_controller: 

    def __init__(self, KP, KI, KD) -> None:

        self.KP = KP
        self.KI = KI
        self.KD = KD

        self.SATURATION = 10        # Saturation for integrative

        self.time = time()
        self.last_time = time()
        self.delta_time = 0

        self.error = 0
        self.last_error = 0
        self.delta_error = 0

        self.integrative = 0



    def proporcional(self): 

        return self.KP * self.error
    


    def integrative(self): 

        self.integrative += self.error * self.delta_time


        if abs(self.integrative) > self.SATURATION: 

            self.integrative = 0
        
        return self.integrative * self.KI
    


    def derivative(self):

        self.delta_error = self.error - self.last_error

        
        if self.delta_error != 0: 

            self.delta_error = self.delta_error / self.delta_time

        else: 

            self.delta_error = 0


        return self.delta_error * self.KD 


    def output(self, error): 

        self.error = error 


        self.time = time()
        self.delta_time = self.time - self.last_time 


        if self.error != 0: 
            
            pid_output = self.proporcional() + self.integrative() + self.derivative()

        else: 

            pid_output = self.proporcional() + self.derivative()

        
        self.last_error = self.error
        self.last_time = self.time


        return pid_output

