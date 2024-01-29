from math import cos, sin, sqrt

g = 9.81
k = 1
m = 10.0

# # Extract current state variables from StateVector
# x_0, y_0, z_0 = 0, 0, 1.7
# x_dot_0, y_dot_0, z_dot_0 = 1.5, 0, 1.75

# h_initial = 1.7

# # Extract control input parameters
# T_duration = 0.4
# r_0, r_T = 2, 1.5

# Extract current state variables from StateVector
x_0, y_0, z_0 = 0.57, 0, 1.611
x_dot_0, y_dot_0, z_dot_0 = 1.5, 0, 1.75

h_initial = 0.57

# Extract control input parameters
T_duration = 0.3
r_0, r_T = 2, 1.5

t = 0

omega = sqrt(k / m)

r = (t/T_duration)*(r_T - r_0) + r_0

d1 = z_0 - r_0 + g/omega**2
d2 = z_dot_0/omega - (r_T - r_0)/(T_duration*omega)
z_stance_cos = cos(omega*t)
z_stance_sin = sin(omega*t)
z_stance_g_omega = (g/omega**2)
z_stance = d1 * cos(omega*t) + d2 * sin(omega*t) + r - (g/omega**2)

print(f"{'Parameter': <15}{'Value': <15}")
print("-" * 30)
print(f"{'g': <15}{g: <15}")
print(f"{'k': <15}{k: <15}")
print(f"{'m': <15}{m: <15}")
print(f"{'x_0': <15}{x_0: <15}")
print(f"{'y_0': <15}{y_0: <15}")
print(f"{'z_0': <15}{z_0: <15}")
print(f"{'x_dot_0': <15}{x_dot_0: <15}")
print(f"{'y_dot_0': <15}{y_dot_0: <15}")
print(f"{'z_dot_0': <15}{z_dot_0: <15}")
print(f"{'h_initial': <15}{h_initial: <15}")
print(f"{'T_duration': <15}{T_duration: <15}")
print(f"{'r_0': <15}{r_0: <15}")
print(f"{'r_T': <15}{r_T: <15}")
print(f"{'t': <15}{t: <15}")
print(f"{'omega': <15}{omega: <15}")
print(f"{'r': <15}{r: <15}")
print(f"{'d1': <15}{d1: <15}")
print(f"{'d2': <15}{d2: <15}")
print(f"{'z_stance_g_o': <15}{z_stance_g_omega: <15}")
print(f"{'z_stance_cos': <15}{z_stance_cos: <15}")
print(f"{'z_stance_sin': <15}{z_stance_sin: <15}")
print(f"{'z_stance': <15}{z_stance: <15}")