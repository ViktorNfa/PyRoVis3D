## Import required packages
import numpy as np
from matplotlib import pyplot as plt

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_sum = np.zeros(3)
        self.prev_error = np.zeros(3)

    def control(self, x, x_goal, dt):
        # Calculate the error
        error = x_goal - x

        # Calculate the proportional term
        p_term = self.kp * error

        # Calculate the integral term
        self.error_sum += error * dt
        i_term = self.ki * self.error_sum

        # Calculate the derivative term
        d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        # Calculate the control signal
        control_signal = p_term + i_term + d_term

        return control_signal
    
class CBFController:
    def __init__(self, pid_controller, obstacle_center, obstacle_radius, radius, lambda_h):
        self.pid_controller = pid_controller
        self.obstacle_center = obstacle_center
        self.obstacle_radius = obstacle_radius
        self.radius = radius
        self.lambda_h = lambda_h

    def compute_safety_barrier(self, x):
        # Compute the safety constraint h(x)
        distance = np.linalg.norm(x - self.obstacle_center)
        h = distance**2 - (self.obstacle_radius+radius)**2
        return h

    def compute_barrier_derivative(self, x):
        # Compute the derivative of the safety constraint h(x)
        distance_vector = x - self.obstacle_center
        h_dot = 2*distance_vector
        return h_dot

    def control(self, x, x_goal, dt):
        # Compute the control signal from the PID controller
        control_signal = self.pid_controller.control(x, x_goal, dt)

        # Compute the safety constraint and its derivative
        h = self.compute_safety_barrier(x)
        h_dot = self.compute_barrier_derivative(x)

        g = 1
        f = 0

        a = - h_dot*g
        b = - h_dot*f - lambda_h * h

        mu = (a*control_signal+b)/(a**2)

        safe_control_signal = control_signal - mu*a

        return safe_control_signal
    
radius = 1

# Parameters for the PID controller
kp = 10
ki = 0
kd = 0

# Maximum and minimum control input
u_min = -10
u_max = 10

# Create a PID controller
pid_controller = PIDController(kp, ki, kd)

# Parameters for the CBF controller
obstacle_center = np.array([5.0, 2.0, 1.0])
obstacle_radius = 1.0
lambda_h = 1.0

# Create a CBF controller
cbf_controller = CBFController(pid_controller, obstacle_center, obstacle_radius, radius, lambda_h)

# Simulation parameters
dt = 0.01  # Time step
t_total = 10.0  # Total simulation time
num_steps = int(t_total / dt)  # Number of simulation steps

# Initial position and goal position
x_initial = np.array([0.0, 0.0, 0.0])
x_goal = np.array([10.0, 5.0, 3.0])

# Simulation loop
y = np.zeros((3,num_steps)) # Vector to save all the positions with time
x = x_initial  # Current position
for step in range(num_steps-1):
    # Compute the safety-enhanced control signal
    control_signal = cbf_controller.control(x, x_goal, dt)
    for i in range(len(control_signal)):
        control_signal[i] = max(u_min, min(u_max, control_signal[i]))

    # Update the position (example: simple integration)
    x += control_signal * dt
    # Save the output of the system
    y[:,step+1] = x

    # Print the current position
    print(f"Step: {step}, Position: {x}, Output: {y[:,step]}")


## Plot evolution of each axis
fig_y, ax_y = plt.subplots()
ax_y.grid(True)
ax_y.plot(range(num_steps), y[0,:], label="X-axis")
ax_y.plot(range(num_steps), y[1,:], label="Y-axis")
ax_y.plot(range(num_steps), y[2,:], label="Z-axis")
ax_y.set_xlabel('time')
ax_y.set_ylabel('y')
ax_y.set_title("Output of PID")
ax_y.legend(fontsize=13)
#ax_y.axhline(y=0, color='k', lw=1)
plt.show()


## Show animation

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

radius = 1
theta = np.linspace(0, 2 * np.pi, 50)
phi = np.linspace(0, np.pi, 25)
theta, phi = np.meshgrid(theta, phi)

# Animation update function
def update(frame, y, ax):
    # Clear the previous plot
    ax.clear()

    # Setting the axes properties
    ax.set_xlim3d([-5.0, 5.0])
    ax.set_xlabel('X')
    ax.set_ylim3d([-5.0, 5.0])
    ax.set_ylabel('Y')
    ax.set_zlim3d([-5.0, 5.0])
    ax.set_zlabel('Z')

    # Plot obstacle
    xo = obstacle_center[0] + obstacle_radius * np.sin(phi) * np.cos(theta)
    yo = obstacle_center[1] + obstacle_radius * np.sin(phi) * np.sin(theta)
    zo = obstacle_center[2] + obstacle_radius * np.cos(phi)

    # Plot the sphere
    obstacle = ax.plot_surface(xo, yo, zo, color='y')

    # Update sphere
    xs = y[0,frame] + radius * np.sin(phi) * np.cos(theta)
    ys = y[1,frame] + radius * np.sin(phi) * np.sin(theta)
    zs = y[2,frame] + radius * np.cos(phi)

    # Plot the sphere
    sphere = ax.plot_surface(xs, ys, zs, color='c')

    # Create the three axes
    axes = ax.quiver(y[0,frame], y[1,frame], y[2,frame], 10, 0, 0, color='r', length=0.2)
    ayes = ax.quiver(y[0,frame], y[1,frame], y[2,frame], 0, 10, 0, color='g', length=0.2)
    azes = ax.quiver(y[0,frame], y[1,frame], y[2,frame], 0, 0, 10, color='b', length=0.2)

    line = ax.plot(y[0, :frame], y[1, :frame], y[2, :frame], color='c')

    return ax

# Animate the plot
def animate():
    # Create a figure and axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for i in range(num_steps):
        ax = update(i,y,ax)
        plt.pause(dt)  # Pause between frames (adjust as needed)

# Start the animation
animate()

# Does not work
#animation = FuncAnimation(fig, update, fargs=(y, ax), frames=num_steps, interval=dt*1000, blit=True)

plt.show()