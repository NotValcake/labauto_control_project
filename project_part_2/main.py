from project_part_2 import RCController, UnbalancedMassMechanicalSys
from labauto import FirstOrderLowPassFilter, Delay
import numpy as np
import matplotlib.pyplot as plt


def main():
        
    Tc = 0.01 # Sampling time
    L = 0.01  # Time delay
    T = 0.01  # Lowpass filter time constant

    m = 1.0
    hv = 1.0
    J = 1.0
    l = 1.0
    g = 9.81

    # Mechanical model creation
    # u = t motor torque
    # x1 = q angular position
    # x2 = qd angular velocity
    # x1d = x2
    # x2d = -hv/J - mlg/J * cos(x1) + u
    # y = x2
    sys = UnbalancedMassMechanicalSys(Tc/2, m, hv, J, l, g)
    sys.initialize()

    # Controller components creation
    delay = Delay(Tc, L)
    lp_filter = FirstOrderLowPassFilter(Tc, T)

    # Controller creation
    rc_controller = RCController(Tc, lp_filter, delay)

    q, qd, qdd, u, t = [], [], [], [], []
    actual_time = 0.0

    for i in range(0, 2000):

        sys.simulate()
        q.append(sys.x[0])
        qd.append(sys.x[1])
        qdd.append(sys.state_function(sys.x, sys.u, actual_time)[1])
        u.append(sys.u)
        t.append(actual_time)

        actual_time += Tc


    time_array = np.array(t)
    q_array = np.array(q)
    qd_array = np.array(qd)
    qdd_array = np.array(qdd)
    u_array = np.array(u)

    fig, axes = plt.subplots(2, 2, figsize=(10, 10))

    # Position
    axes[0, 0].plot(time_array, q_array[:], label='Position')

    # Velocity
    axes[1, 0].plot(time_array, qd_array[:], label='Velocity')

    # Acceleration
    axes[0, 1].plot(time_array, qdd_array[:], label='Acceleration')

    # Torque (Control input)
    axes[1, 1].plot(time_array, u_array[:], label='Torque')

    # Formatting
    for ax in axes.flat:
        if ax.has_data():
            ax.grid()
            ax.legend()
            ax.set_xlabel("Time (s)")

    plt.tight_layout()
    plt.show()

if __name__=="__main__":
    main()