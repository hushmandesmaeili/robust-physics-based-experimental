import sympy as sp
import matplotlib.pyplot as plt
import numpy as np

# Symbolic variables
t = sp.symbols('t')
z = sp.Function('z')
d1, d2, z0, r0, g, omega, k, m, zdot0, rT, T = sp.symbols('d1 d2 z0 r0 g omega k m zdot0 rT T')

# Given expressions
d1_expr = z0 - r0 + g / omega**2
d2_expr = (zdot0 / omega) - (rT - r0) / (T * omega)
omega_expr = sp.sqrt(k / m)
r = (rT - r0) / T + r0

# Equation for z(t)
z_expr = d1 * sp.cos(omega * t) + d2 * sp.sin(omega * t) + r - g / omega**2

# Substitute given expressions into the equation
z_expr = z_expr.subs({d1: d1_expr, d2: d2_expr, omega: omega_expr})
print(z_expr)

# Convert the symbolic expression to a function
z_func = sp.lambdify((t, z0, r0, g, k, m, zdot0, rT, T), z_expr, 'numpy')

# Generate time values for plotting
time_values = np.linspace(0, 10, 1000)

# Set values for the other parameters
z0_val = 1.0
r0_val = 0.5
g_val = 9.8
k_val = 5.0
m_val = 1.0
zdot_val = 0.2
rT_val = 0.8
T_val = 2.0

# Evaluate the function for the given parameter values
z_values = z_func(time_values, z0=z0_val, r0=r0_val, g=g_val, k=k_val, m=m_val, zdot0=zdot_val, rT=rT_val, T=T_val)

# Plot the function
plt.plot(time_values, z_values)
plt.xlabel('Time (s)')
plt.ylabel('z(t)')
plt.title('Plot of the equation z(t)')
plt.grid(True)
plt.show()
