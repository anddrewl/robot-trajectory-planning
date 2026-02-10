from mobile_manipulator_unicycle import MobileManipulatorUnicycle
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

robot = MobileManipulatorUnicycle(robot_id=1, backend_server_ip="192.168.0.2")

print("Move forward for 2 seconds and set the LEDs red")
start_time = time.time()
while time.time() - start_time < 2.:
    robot.set_mobile_base_speed_and_gripper_power(v=0.5, omega=0.0, gripper_power=0.0)
    robot.set_leds(255, 0, 0)
    time.sleep(0.05)

print("Stop the base and the gripper")
robot.set_mobile_base_speed_and_gripper_power(0., 0., 0.)

print("Get the robot's current pose")
poses = robot.get_poses()
print(f"Robot, pickup, dropoff, obstacles poses: {poses}")

x_i, y_i, theta_i = robot.robot_pose
x_g, y_g, theta_g = robot.dropoff_pose

theta_i = 0
theta_g = -np.pi / 6

t = np.array([0, 20])
t_dense = np.linspace(t[0], t[1], 500)

speed = np.linalg.norm([x_g - x_i, y_g - y_i]) / t[1]
v_i = speed * np.array([np.cos(theta_i), np.sin(theta_i)])
v_g = speed * np.array([np.cos(theta_g), np.sin(theta_g)])

spline_x = CubicSpline(t, [x_i, x_g], bc_type=((1, v_i[0]), (1, v_g[0])))
spline_y = CubicSpline(t, [y_i, y_g], bc_type=((1, v_i[1]), (1, v_g[1])))

x_vals = spline_x(t_dense)
y_vals = spline_y(t_dense)
dx_vals = spline_x.derivative()(t_dense)
dy_vals = spline_y.derivative()(t_dense)
ddx_vals = spline_x.derivative(2)(t_dense)
ddy_vals = spline_y.derivative(2)(t_dense)
theta_vals = np.arctan2(dy_vals, dx_vals)

v_vals = np.sqrt(dx_vals**2 + dy_vals**2)
omega_vals = (ddy_vals * dx_vals - ddx_vals * dy_vals) / (dx_vals**2 + dy_vals**2 + 1e-6)

zdot_prev = np.array([dx_vals[0], dy_vals[0]])
z_prev = np.array([x_i, y_i])
t_prev = t_dense[0]

kp = 2
kd = 4

A = np.array([[0, 0, 1, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0],
              [0, 0, 0, 0]])
B = np.array([[0, 0],
              [0, 0],
              [1, 0],
              [0, 1]])

v_feedback = []
z_feedback = []
theta_feedback = []
omega_feedback = []
t_feedback = []

x_actual = []
y_actual = []

dt = 0.05
task_done = False

fig, ax = plt.subplots(figsize=(8, 6))
ax.plot(x_vals, y_vals, 'orange', linewidth=2, label='Planned Path')
ax.plot(x_i, y_i, 'go', label='Start')
ax.plot(x_g, y_g, 'ro', label='Goal')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('Spline Path')
ax.legend()
ax.axis('equal')
ax.grid(True)
plt.show(block=True)

while not task_done:
    for i in range(len(x_vals)):
        real_pose = robot.robot_pose
        z = np.array([real_pose[0], real_pose[1]])
        theta = real_pose[2]

        z_d = np.array([x_vals[i], y_vals[i]])
        zdot_d = np.array([dx_vals[i], dy_vals[i]])
        zddot_d = np.array([ddx_vals[i], ddy_vals[i]])

        zdot = (z - z_prev) / dt
        zddot = (zdot - zdot_prev) / dt

        u_w = kp * (z_d - z) + kd * (zdot_d - zdot)
        w = np.hstack([z, zdot])
        wdot = A @ w + B @ u_w

        w = w + wdot * dt
        wdot_new = w[2:4]
        wddot_new = wdot[2:4]

        v = np.linalg.norm(wdot_new)
        omega = (wddot_new[1] * wdot_new[0] - wddot_new[0] * wdot_new[1]) / (wdot_new[0]**2 + wdot_new[1]**2 + 1e-6)

        robot.set_mobile_base_speed_and_gripper_power(v, omega, 0.0)

        z_prev = z.copy()
        zdot_prev = zdot.copy()
        t_prev = t_dense[i]

        v_feedback.append(v)
        omega_feedback.append(omega)
        t_feedback.append(t_dense[i])

        x_actual.append(z[0])
        y_actual.append(z[1])

        time.sleep(dt)

        if i == 499:
            task_done = True

v_feedback = np.array(v_feedback)
omega_feedback = np.array(omega_feedback)
t_feedback = np.array(t_feedback)

x_actual = np.array(x_actual)
y_actual = np.array(y_actual)

print(robot.robot_pose)
print(robot.dropoff_pose)

print("Stop the base and the gripper")
robot.set_mobile_base_speed_and_gripper_power(0., 0., 0.)

# === Plot actual vs planned path ===
plt.figure(figsize=(8, 6))
plt.plot(x_vals, y_vals, color='orange', linestyle=':', label='Planned Path (x, y)')
plt.plot(x_actual, y_actual, color='blue', linestyle='-', label='Actual Path (x, y)')
plt.plot(x_i, y_i, 'go', label='Start')
plt.plot(x_g, y_g, 'ro', label='Goal')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Actual vs Planned Path in XY Plane')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()

# === Save paths ===
planned_path = np.column_stack((x_vals, y_vals))
np.savetxt("planned_path.txt", planned_path, fmt="%.6f", header="x y", comments='')
print("Planned path saved to planned_path.txt")

actual_path = np.column_stack((x_actual, y_actual))
np.savetxt("actual_path.txt", actual_path, fmt="%.6f", header="x y", comments='')
print("Actual path saved to actual_path.txt")

# === Plot velocity comparison ===
plt.figure(figsize=(10, 4))
plt.plot(t_dense, v_vals, 'b--', label='v (Feedforward)')
plt.plot(t_feedback, v_feedback, 'b-', label='v (Feedback)')
plt.xlabel('Time [s]')
plt.ylabel('Linear Velocity v [m/s]')
plt.title('Linear Velocity: Feedforward vs Feedback')
plt.legend()
plt.grid(True)
plt.show(block=False)
plt.pause(0.1)

plt.figure(figsize=(10, 4))
plt.plot(t_dense, omega_vals, 'r--', label='ω (Feedforward)')
plt.plot(t_feedback, omega_feedback, 'r-', label='ω (Feedback)')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity ω [rad/s]')
plt.title('Angular Velocity: Feedforward vs Feedback')
plt.legend()
plt.grid(True)
plt.show(block=False)
plt.pause(0.1)

input("Press Enter to close all plots...")
plt.close('all')
