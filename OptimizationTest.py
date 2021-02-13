import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d


# Simulation Parameters
# =====================
dt = 0.01

# Tube Parameters
# ===============
tube_length = 1500
tube_brake = 200 # Distance remaining at the end post braking
tube_allow = tube_length - tube_brake

def run_iteration(pod_mass, wheel_d_in):

    """
    Arguments:
    Takes in an expected mass and a wheel diameter in inches.

    Returns:
    Max Speed (Just before braking)
    """
    # Simulation Variable Init.
    podV = [0]
    podX = [0]
    podA = []
    
    # Battery Parameters
    bat_n = 1  # Batteries in parallel
    bat_ser = 14  # Num Cells In Pack
    bat_v = bat_ser * 4.2
    cell_r = 0.012 # Internal resistance per cell
    current_max = 300
    bat_r = bat_ser * cell_r / bat_n
    cur_at_max = bat_v / (2 * bat_r)

    # Drive Parameters
    # Specifying Motor Params.
    # ================
    num_motors = 4
    wheel_r = wheel_d_in * 0.0127  # Radius in meters, commented out for optimization
    w_max = 52000 * 0.104719755  # RPM to rad/s at nominal voltage
    w_max_v = 6 * 4.2
    kv = w_max / w_max_v
    kt = 0.010913423831071  # N*m/Amp derived from Excel
    tau_max = 6

    # Brake Properties
    brake_accel = -3 * 9.8  # Braking magnitude in m/s^2

    # Mass Properties
    # Commented out for optimization
    # pod_mass = 100

    # Gets the expected torque from the motor at a given angular velocity
    def get_tau(w):
        return tau_max - (tau_max / w_max) * w

    # Gets pod acceleration based on pod velocity
    def get_accel(v):
        return (num_motors * get_tau(v / wheel_r) / wheel_r) / pod_mass

    # Acceleration Phase
    while True:
        # Extended Euler Ingration (Heun's Method)
        a = 0.5 * (get_accel(podV[-1]) + get_accel(podV[-1] +
                get_accel(podV[-1]) * dt))
        podV.append(podV[-1] + a * dt)
        podX.append(podX[-1] + podV[-1] * dt)
        podA.append(a)

        # Compute braking distance for use in brake trigger
        brake_dis = (podV[-1] * podV[-1]) / (2 * abs(brake_accel))
        rem_dis = tube_allow - podX[-1]
        
        # Braking trigger
        if brake_dis >= rem_dis:
            podX_stop = podX[-1]
            # print('Braking Start at %0.2f m/s or %0.2f mph' % (podV[-1], podV[-1]*2.23))
            break

    # Braking Phase
    while True:
        podV.append(podV[-1] + brake_accel * dt)
        podX.append(podX[-1] + podV[-1] * dt)
        podA.append(brake_accel)

        # If the pod stops, run is over.
        if podV[-1] <= 0:
            # print('Run end at %i meters' % podX[-1])
            break

    return max(podV)


# Run Optimization
masses = np.linspace(80,120,30)
diams = np.linspace(1,6,30)

# Init Data Matrix
D = np.zeros((masses.size, diams.size))

# Run Iterations
for i,m in enumerate(masses):
    for j,d in enumerate(diams):
        D[i,j] = run_iteration(m,d)

# Plot Optimization Surface
M, W = np.meshgrid(diams, masses)
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot_surface(M, W, D, rstride=1, cstride=1,
                cmap='viridis', edgecolor='none')
ax.set_title('Pod Optimization');
ax.set_ylabel('Mass (kg)')
ax.set_xlabel('Wheel Diameter (in.)')
ax.set_zlabel('V (m/s)')
plt.show()