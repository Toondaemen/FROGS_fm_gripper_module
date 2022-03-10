#!/usr/bin/env python3

import time

class PIDController():
    def __init__(self, f_sample, P=0.1, I=0.01, D=0.1):

        self.Pgain = P
        self.Igain = I
        self.Dgain = D

        if f_sample != 0:
            self.sample_time = 1/f_sample
        else:
            self.sample_time = 0

        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        self.last_error = 0
        self.int_error = 0
        self.windup_guard = 1

    def control(self, set_point, measurement):
        error = set_point - measurement

        self.current_time = time.time()
        delta_t = self.current_time - self.last_time

        delta_error = error - self.last_error

        Pterm = 0
        Iterm = 0
        Dterm = 0

        if delta_t > self.sample_time:
            Pterm = self.Pgain*error
            self.int_error += self.Igain*error*delta_t

            if self.int_error < -self.windup_guard:
                Iterm = -self.windup_guard
            elif self.int_error > self.windup_guard:
                Iterm = self.windup_guard
            else:
                Iterm = self.int_error

            if delta_t > 0:
                Dterm = self.Dgain*delta_error/delta_t

            self.last_time = self.current_time
            self.last_error = error

        return Pterm + Iterm + Dterm
