from project_part_2 import RCController, UnbalancedMassMechanicalSys
from labauto import FirstOrderLowPassFilter, Delay
import numpy as np
import matplotlib.pyplot as plt


def main():
    w = 10 # rad/s input signal
    sampling_time = 0.005 # Sampling time
    delay_filter_dead_time = np.round(6.28/w,1) # Time delay
    lpf_time_constant = 0.021 # Lowpass filter time constant
    # LPF = 1
    #      ---
    #    1 + sT

    mass = 0.4          # m
    damping_coeff = 1.0 # hv
    inertia = 1.0       # J
    link_len = 0.5      # l
    g = 9.81

    # Mechanical model creation
    # u = t motor torque
    # x1 = q angular position
    # x2 = qd angular velocity
    # x1d = x2
    # x2d = -hv/J - mlg/J * cos(x1) + u
    # y = x2
    sys = UnbalancedMassMechanicalSys(sampling_time, mass, damping_coeff, inertia, link_len, g)
    sys.initialize()
    sys.starting()

    # Controller components creation
    delay = Delay(sampling_time, delay_filter_dead_time)
    lp_filter = FirstOrderLowPassFilter(sampling_time, lpf_time_constant)

    # Controller creation
    rc_controller = RCController(sampling_time, lp_filter, delay)
    rc_controller.initialize()
    rc_controller.starting(0, 0, 0)
    q, qd, qdd, u, t = [], [], [], [], []
    actual_time = 0.0

    # Free system evolution
    # for i in range(0, 2000):
    #     sys.simulate()
    #     q.append(sys.x[0])
    #     qd.append(sys.x[1])
    #     qdd.append(sys.state_function(sys.x, sys.u, actual_time)[1])
    #     u.append(sys.u)
    #     t.append(actual_time)
    #     actual_time += sampling_time

    # time_array = np.array(t)
    # q_array = np.array(q)
    # qd_array = np.array(qd)
    # qdd_array = np.array(qdd)
    # u_array = np.array(u)

    # fig, axes = plt.subplots(2, 2, figsize=(10, 10))

    # # Position
    # axes[0, 0].plot(time_array, q_array[:], label='Position')

    # # Velocity
    # axes[1, 0].plot(time_array, qd_array[:], label='Velocity')

    # # Acceleration
    # axes[0, 1].plot(time_array, qdd_array[:], label='Acceleration')

    # # Torque (Control input)
    # axes[1, 1].plot(time_array, u_array[:], label='Torque')

    # # Formatting
    # for ax in axes.flat:
    #     if ax.has_data():
    #         ax.grid()
    #         ax.legend()
    #         ax.set_xlabel("Time (s)")

    # plt.tight_layout()
    # plt.show()

    # Controlled system output (1st spec: 20 seconds qd = 10 rad/s)

    q, qd, qdd, u, t , ref= [0], [0], [0], [0], [0], [0]
    actual_time = 0.0

    
    total_time = 20


    for i in range(0, (int)(total_time/sampling_time)):
        r = np.sin(w * actual_time)
        uc = rc_controller.compute_control_action(r, sys.read_sensor_value())
        sys.write_actuator_value([uc])
        sys.simulate()
        q.append(sys.x[0])
        qd.append(sys.x[1])
        qdd.append(sys.state_function(sys.x, [uc], actual_time)[1])
        u.append(uc)
        t.append(actual_time)
        ref.append(r)

        actual_time += sampling_time

    time_array = np.array(t)
    q_array = np.array(q)
    qd_array = np.array(qd)
    qdd_array = np.array(qdd)
    u_array = np.array(u)
    r_array = np.array(ref)

    fig, axes = plt.subplots(2, 2, figsize=(10, 10))

    # Error
    axes[0, 0].plot(time_array,(r_array[:] - qd_array[:]), label='Velocity error')
    axes[0, 0].legend(loc='best')

    # Velocity
    axes[1, 0].plot(time_array, r_array[:] ,label = 'reference signal w = 10rad/s')
    axes[1, 0].plot(time_array, qd_array[:], label='Controlled velocity')
    axes[1, 0].legend(loc='best')

    # Acceleration
    axes[0, 1].plot(time_array, qdd_array[:], label='Acceleration')

    # Torque (Control input)
    axes[1, 1].plot(time_array, u_array[:], label='Control Input')
    axes[1,1].legend(loc = 'best')

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