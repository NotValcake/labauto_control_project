from labauto import MechanicalSystem
import numpy as np
import math

class UnbalancedMassMechanicalSys(MechanicalSystem):

    def __init__(self, st, m: float, hv: float, J: float, l:float, g:float):
        """Constructor of UnbalancedMassMechanicalSys.

        :param st: sampling time
        :param m: mass
        :type m: float
        :param hv: viscous friction coefficientviscous friction coefficient
        :type hv: float
        :param J: inertia moment
        :type J: float
        :param l: link length
        :type l: float
        :param g: gravitational acceleration
        :type g: float
        """
        self.m = m
        self.hv = hv
        self.J = J
        self.l = l
        self.g = g
        super().__init__(st)

    def starting(self):
        """
        Set the equilibrium position as starting position.
        """
        self.x[0] = 3/2*np.pi
        self.x[1] = 0

    def state_function(self, x: np.array, u: float, t)->np.ndarray:
        """ Definition of the state transition function

        x1d = x2
        x2d = -hv/J - mlg/J * cos(x1) + u

        :param x: x1 = q angular position
                  x2 = qd angular velocity
        :type x: np.array
        :param u: motor torque
        :type u: float
        :param t: not used, here just to enforce override of the function instead of overload
        :return: x1d = x2 = qd (velocity of the motor)
                 x2d = qdd (acceleration of the motor)
        :rtype: np.ndarray
        """
        return np.array([
                         x[1],
                         -self.hv/self.J * x[1] - self.m*self.l*self.g/self.J * math.cos(x[0]) + u[0]
                        ])
    
    def output_function(self)->float:
        """
        Definition of the output function.
        :return: y = x2 = qd (velocity of the motor)
        """
        return self.x[1]