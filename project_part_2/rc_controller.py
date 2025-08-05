from labauto import BaseController
import numpy as np

from labauto import FirstOrderLowPassFilter
from labauto import Delay

class RCController(BaseController):
    """
    This class implements a repetitive controller
    """


    def __init__(self, Tc: float, LPFilter: FirstOrderLowPassFilter, Delay: Delay):
        """
        Initialize the repetitive modified compensator

        :param Tc: Sampling time (must be a positive scalar).
        :param LPFilter: First order lowpass filter.
        :param Delay: Delay.
        """
        super().__init__(Tc)
        self.LPFilter = LPFilter
        self.Delay = Delay


    def initialize(self):
        """Initialize the state variables."""
        self.Delay.initialize()
        self.LPFilter.initialize()
        self.u = 0.0


    def starting(self, reference: float, measure: float, u: float):
        """
        Start the repetitive controller

        :param reference: Reference input signal fo the controller
        :param measure: Measured process output
        :param u: Iniitial control action
        """
        self.u = u
        initial_error = reference - measure
        self.LPFilter.starting(initial_error)
        self.Delay.starting(initial_error)


    def compute_control_action(self, reference: float, y: float) -> float:
        """
        Computes the repetitive controller control action

        :param reference: Reference input signal fo the controller
        :param y: Process output
        """
        error = reference - y
        v = self.LPFilter.step(error + self.u)  # uscita del LPF
        u = self.Delay.step(v)  # uscita del delay
        self.u = u
        return self.u