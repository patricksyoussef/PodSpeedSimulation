from matplotlib import pyplot as plt
from matplotlib import animation

# Helper Function for Clearer Prints
def printBreak(text):
    n = len(text)
    print('# ' + '=' * n)
    print('# ' + text)
    print('# ' + '=' * n + '\n')


# Initial print statements
# ========================
printBreak('Running pod simulation for HyperXite 5!')

# Simulation Parameters
# =====================
dt = 0.01
save_vid = True

# Simulation Variable Init.
podV = [0]
podX = [0]
podA = []

# Print Sim. Params.
print('# Simulation Parameters')
print('Sim Timestep: %0.4f seconds' % dt)

# Tube Parameters
# ===============
tube_length = 1500
tube_brake = 200 # Distance remaining at the end post braking
tube_allow = tube_length - tube_brake
print('# Tube Parameters')
print('Operating Length %0.0f\n' % tube_allow)

# Battery Parameters
bat_n = 1  # Batteries in parallel
bat_ser = 14  # Num Cells In Pack
bat_v = bat_ser * 4.2
cell_r = 0.012 # Internal resistance per cell
current_max = 300
bat_r = bat_ser * cell_r / bat_n
cur_at_max = bat_v / (2 * bat_r)

# Print Battery Params.
print('# Battery Parameters')
print('Parallel Banks: %i' % bat_n)
print('Series Cells: %i' % bat_ser)
print('Series Voltage: %0.1f' % bat_v)
print('Cell Resistance: %0.2f mOhms' % (cell_r * 1000))
print('Array Resistance: %0.2f mOhms\n' % (bat_r * 1000))

# Drive Parameters
# Specifying Motor Params.
# ================
num_motors = 4
wheel_r = 3.0 * 0.0127  # Radius in meters
w_max = 52000 * 0.104719755  # RPM to rad/s at nominal voltage
w_max_v = 6 * 4.2
kv = w_max / w_max_v
kt = 0.010913423831071  # N*m/Amp derived from Excel
tau_max = 6

# Brake Properties
brake_accel = -3 * 9.8  # Braking magnitude in m/s^2

# Mass Properties
pod_mass = 100  # Mass in kilos

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
        print('Braking Start at %0.2f m/s or %0.2f mph'
            % (podV[-1], podV[-1]*2.23))
        break

# Braking Phase
while True:
    podV.append(podV[-1] + brake_accel * dt)
    podX.append(podX[-1] + podV[-1] * dt)
    podA.append(brake_accel)

    # If the pod stops, run is over.
    if podV[-1] <= 0:
        print('Run end at %i meters' % podX[-1])
        break

# Record final states
lastX = podX[-1]
podA.append(0)
for i in range(500):
    podX.append(lastX)
    podV.append(0)
    podA.append(0)

# Run mp4 save of velocity over distance
if save_vid:
    vidX = []
    vidV = []
    vidA = []
    vid_step = 20
    for i in range(0, len(podV)):
        if i % vid_step == 0:
            vidX.append(podX[i])
            vidV.append(podV[i])
            vidA.append(podA[i])

    # First set up the figure, the axis, and the plot element we want to animate
    fig = plt.figure()
    ax = plt.axes(xlim=(0, tube_length), ylim=(0, 1.5 * max(podV)))
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
    accel_text = ax.text(0.02, 0.9, '', transform=ax.transAxes)
    ax.set_title('Pod Velocity vs Tube Length')
    ax.set_xlabel('Position (m)')
    ax.set_ylabel('Velocity (m/s)')
    line, = ax.plot([], [], lw=2)

    # Print lines for brake stop and tube end.
    ax.vlines(podX_stop, 0, 1.5 * max(podV), linestyle="dashed", color="orange")
    ax.text(podX_stop, 1.1 * max(podV), 'Brake Start', 
    rotation=90, fontsize=10, horizontalalignment='right', 
    verticalalignment="bottom")
    ax.vlines(tube_length, 0, 1.5 * max(podV), linestyle="dashed", color="red")
    ax.text(tube_length, 1.1 * max(podV), 'Tube End', 
    rotation=90, fontsize=10, horizontalalignment='right', 
    verticalalignment="bottom")
    ax.vlines(tube_length-tube_brake, 0, 1.5 * max(podV), color="green")
    ax.text(tube_length-tube_brake, 1.1 * max(podV), 'Allowable', 
    rotation=90, fontsize=10, horizontalalignment='right',
    verticalalignment="bottom")

    # initialization function: plot the background of each frame
    def init():
        line.set_data([], [])
        time_text.set_text('')
        accel_text.set_text('')
        return line, time_text, accel_text

    # Animate Function, advances line and text
    def animate(i):
        x = vidX[0:i]
        y = vidV[0:i]
        time = i*dt*vid_step
        time_text.set_text('time = %.2f s' % time)
        accel_text.set_text('accel = %.2f m/s^2' % vidA[i])
        line.set_data(x, y)
        return line, time_text, accel_text

    # Iterates through list vidV
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=len(vidV), interval=vid_step * dt, blit=False)

    anim.save('pod_run.mp4', fps=30, extra_args=['-vcodec', 'libx264'], dpi=200)

    plt.show()