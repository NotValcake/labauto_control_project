from project_part_2 import RCController, UnbalancedMassMechanicalSys
from labauto import FirstOrderLowPassFilter, Delay
import numpy as np
import matplotlib.pyplot as plt
from time import perf_counter

def main():
    sampling_time = 0.005

   # Controller parameters
    delay_filter_dead_time = 0.5
    lpf_time_constant = 3

    # System parameters
    mass = 0.4
    damping_coeff = 1
    link_len = 0.5
    inertia = 1.2
    g = 9.81

    # Creation of the controlled system
    sys = UnbalancedMassMechanicalSys(sampling_time, mass, damping_coeff, inertia, link_len, g)
    sys.initialize()
    sys.starting()

    # Creation of controller components
    delay = Delay(sampling_time, delay_filter_dead_time)
    lp_filter = FirstOrderLowPassFilter(sampling_time, lpf_time_constant)

    # Creation and initialization of the RC controller
    rc_controller = RCController(sampling_time, lp_filter, delay)
    rc_controller.initialize()
    rc_controller.starting(0, 0, 0)

    # Initialization of empty lists
    position, velocity, acceleration, control_actions, t, reference = [0], [0], [0], [0], [0], [0]
    actual_time = 0.0

    time_intervals = [20.0, 20.0, 20.0]          # three 20 s intervals
    intervals_reference    = [10.0, 12.5, 15.0]  # per-interval reference angular velocity

    # Initialize maximum cycle time counter
    max_cycle_time = -1.0;

    for i in range(len(time_intervals)):
        # Perform the simulation loop for each time step of each time interval
        for _ in range(int(time_intervals[i] / sampling_time)):
            tic = perf_counter()
            actual_reference = intervals_reference[i]

            system_output = sys.read_sensor_value()
            actual_control_action = rc_controller.compute_control_action(actual_reference, system_output)
            sys.write_actuator_value([actual_control_action])
            sys.simulate()

            position.append(sys.x[0])
            velocity.append(sys.x[1])
            acceleration.append(sys.state_function(sys.x, [actual_control_action], actual_time)[1])
            control_actions.append(actual_control_action)
            t.append(actual_time)
            reference.append(actual_reference)

            actual_time += sampling_time
            toc = perf_counter()

            # Store maximum cycle time
            max_cycle_time = max(toc-tic, max_cycle_time)

    # Print maximum cycle time
    print(f"Maximum cycle time: {max_cycle_time} s")

    # Convert lists to np.array
    time_array = np.array(t)
    velocity_array = np.array(velocity)
    acceleration_array = np.array(acceleration)
    control_actions_array = np.array(control_actions)
    reference_array = np.array(reference)

    # Visualization
    fig, axes = plt.subplots(2, 2, figsize=(10, 10))

    axes[0, 0].plot(time_array, (reference_array - velocity_array), label='Velocity error')
    axes[0, 0].set_title("Velocity error")
    axes[0, 0].set_ylabel("velocity error [rad/s]")
    axes[0, 0].legend(loc='best')
    axes[0, 0].grid(True)

    axes[1, 0].plot(time_array, reference_array, label='Reference')
    axes[1, 0].plot(time_array, velocity_array, label='Output')
    axes[1, 0].set_title("Reference vs. actual output")
    axes[1, 0].set_ylabel("velocity [rad/s]")
    axes[1, 0].legend(loc='best')
    axes[1, 0].grid(True)

    axes[0, 1].plot(time_array, acceleration_array, label='Acceleration')
    axes[0, 1].set_title("Motor acceleration")
    axes[0, 1].set_ylabel("acceleration [rad/s^2]")
    axes[0, 1].legend(loc='best')
    axes[0, 1].grid(True)

    axes[1, 1].plot(time_array, control_actions_array, label='Control Input')
    axes[1, 1].set_title("Motor velocity")
    axes[1, 1].set_ylabel("torque [Nm]")
    axes[1, 1].legend(loc='best')
    axes[1, 1].grid(True)

    for ax in axes.flat:
        ax.set_xlabel("Time [s]")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
