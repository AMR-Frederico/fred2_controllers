#!/usr/bin/env python3

from time import time 

class PID_controller: 

    def __init__(self, KP, KI, KD, reset_cmd) -> None:

        self.KP = KP
        self.KI = KI
        self.KD = KD

        self.SATURATION = 15        # Saturation for integrative

        self.time = time()
        self.last_time = time()
        self.delta_time = 0.0

        self.error = 0.0
        self.last_error = 0.0
        self.delta_error = 0.0

        self.integral = 0.0

        self.reset = reset_cmd


    def gain_scheduling(self, KP, KI, KD):

        self.KP = KP
        self.KI = KI
        self.KD = KD



    def proportional(self): 

        return self.KP * self.error
    



    def integrative(self): 

        self.integral += self.error * self.delta_time


        if abs(self.integral*self.KI) > self.SATURATION or self.reset: 

            self.integral = 0


        return self.integral * self.KI
    



    def derivative(self):

        self.delta_error = self.error - self.last_error

        
        if self.delta_error != 0.0: 

            self.delta_error = self.delta_error / self.delta_time

        else: 

            self.delta_error = 0.0


        return self.delta_error * self.KD 




    def output(self, error): 

        self.error = error 


        self.time = time()
        self.delta_time = self.time - self.last_time 


        if self.error != 0 or abs(self.error) > 0.1: 
            
            pid_output = self.proportional() + self.integrative() + self.derivative()


        else: 

            pid_output = self.proportional() + self.derivative()

            self.integral = 0


        
        self.last_error = self.error
        self.last_time = self.time


        return pid_output

