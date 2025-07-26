from labauto import MechanicalSystem
import numpy as np
import math

class UnbalancedMassMechanicalSys(MechanicalSystem):

    def __init__(self, st, m: float, hv: float, J: float, l:float, g:float):
        self.m = m
        self.hv = hv
        self.J = J
        self.l = l
        self.g = g
        super().__init__(st)

    def state_function(self, x: np.array, u: float, t)->np.ndarray:
        """
        Definition of the state transition function
        x1d = x2
        x2d = -hv/J - mlg/J * cos(x1) + u

        :param u: motor torque
        :param x: x1 = q angular position
                  x2 = qd angular velocity
        """
        return np.array([
                         x[1],
                         -self.hv/self.J * x[1] - self.m*self.l*self.g/self.J * math.cos(x[0]) + u[0]
                        ])
    
    def output_function(self)->float:
        """
        Definition of the output function.
        y = x1 = qd
        """
        return self.x[1]