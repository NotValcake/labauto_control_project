from project_part_2 import RCController, UnbalancedMassMechanicalSys
from labauto import FirstOrderLowPassFilter, Delay
import numpy as np
import matplotlib.pyplot as plt

def main():
    sampling_time = 0.005
    delay_filter_dead_time = 0.5
    lpf_time_constant = 3.14

    mass = 0.4
    damping_coeff = 1.0
    inertia = 1.0
    link_len = 0.5
    g = 9.81

    sys = UnbalancedMassMechanicalSys(sampling_time, mass, damping_coeff, inertia, link_len, g)
    sys.initialize()
    sys.starting()

    delay = Delay(sampling_time, delay_filter_dead_time)
    lp_filter = FirstOrderLowPassFilter(sampling_time, lpf_time_constant)

    rc_controller = RCController(sampling_time, lp_filter, delay)
    rc_controller.initialize()
    rc_controller.starting(0, 0, 0)

    q, qd, qdd, u, t, ref = [0], [0], [0], [0], [0], [0]
    actual_time = 0.0


    time_period = [20.0, 20.0, 20.0]        # tre periodi da 20 s
    reference_w    = [10.0, 12.5, 15.0]        # per ogni blocco

    for i in range(len(time_period)):
        for _ in range(int(time_period[i] / sampling_time)):
            r = reference_w[i]  #reference

            y = sys.read_sensor_value()
            uc = rc_controller.compute_control_action(r, y)
            sys.write_actuator_value([uc])
            sys.simulate()

            q.append(sys.x[0])
            qd.append(sys.x[1])
            qdd.append(sys.state_function(sys.x, [uc], actual_time)[1])
            u.append(uc)
            t.append(actual_time)
            ref.append(r)

            actual_time += sampling_time
    # --- FINE ---

    time_array = np.array(t)
    qd_array = np.array(qd)
    qdd_array = np.array(qdd)
    u_array = np.array(u)
    r_array = np.array(ref)

    fig, axes = plt.subplots(2, 2, figsize=(10, 10))

    axes[0, 0].plot(time_array, (r_array - qd_array), label='Velocity error')
    axes[0, 0].legend(loc='best'); axes[0, 0].grid(True)

    axes[1, 0].plot(time_array, r_array, label='reference')
    axes[1, 0].plot(time_array, qd_array, label='output')
    axes[1, 0].legend(loc='best'); axes[1, 0].grid(True)

    axes[0, 1].plot(time_array, qdd_array, label='Acceleration')
    axes[0, 1].legend(loc='best'); axes[0, 1].grid(True)

    axes[1, 1].plot(time_array, u_array, label='Control Input')
    axes[1, 1].legend(loc='best'); axes[1, 1].grid(True)

    for ax in axes.flat:
        ax.set_xlabel("Time (s)")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
