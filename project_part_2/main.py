from RCController import RCController

from labauto import FirstOrderLowPassFilter
from labauto import Delay

Tc = 0.01
L = 0.01
T = 0.01

delay = Delay(Tc, L)
lp_filter = FirstOrderLowPassFilter(Tc, T)

rc_controller = RCController(Tc, lp_filter, delay)