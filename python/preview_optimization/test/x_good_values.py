from math import cos, exp, sin, sqrt

g = 9.81
k = 1
m = 10.0

# Extract current state variables from StateVector
x_0, y_0, z_0 = 0, 0, 1.5
x_dot_0, y_dot_0, z_dot_0 = 1, 1, 0

h_initial = 1.5

# Extract control input parameters
T_duration = 0.25
r_0, r_T = 1.75, 2
p_0, p_T = 0, 1
t = 0.5

p_x = (t/T_duration)*(p_T - p_0) + p_0

alpha_freq = sqrt(g / h_initial)

# Compute stance dynamics
beta_x_1 = ((x_0 - p_0)/2) + ((x_dot_0*T_duration - (p_T - p_0)) / (2*alpha_freq*T_duration))
beta_x_2 = ((x_0 - p_0)/2) - ((x_dot_0*T_duration - (p_T - p_0)) / (2*alpha_freq*T_duration))

x_stance_exp1 = exp(alpha_freq*t)
x_stance_exp2 = exp(-alpha_freq*t)
x_stance = beta_x_1 * exp(alpha_freq*t) + beta_x_2 * exp(-alpha_freq*t) + p_x

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
print(f"{'p_0': <15}{p_0: <15}")
print(f"{'p_T': <15}{p_T: <15}")
print(f"{'t': <15}{t: <15}")
print(f"{'p_x': <15}{p_x: <15}")
print(f"{'alpha_freq': <15}{alpha_freq: <15}")
print(f"{'beta_x_1': <15}{beta_x_1: <15}")
print(f"{'beta_x_2': <15}{beta_x_2: <15}")
print(f"{'x_stance_exp1': <15}{x_stance_exp1: <15}")
print(f"{'x_stance_exp2': <15}{x_stance_exp2: <15}")
print(f"{'x_stance': <15}{x_stance: <15}")