from sympy import Function, dsolve, Eq, Derivative, symbols, latex

# Define the variables and functions
t, T, r_T, r_0, k, m, g, z0, zdot0 = symbols('t T r_T r_0 k m g z0 zdot0')
z = Function('z')
r = (t * (r_T - r_0) / T) + r_0

# Define the differential equation
ode = Eq(m * Derivative(z(t), t, t), k * (r - z(t)) - m * g)

# Define the initial conditions as symbolic expressions
initial_conditions = {z(0): z0, Derivative(z(t), t).subs(t, 0): zdot0}

# Solve the differential equation
solution = dsolve(ode, z(t), ics=initial_conditions)

# Display the LaTeX-formatted solution
latex_solution = latex(solution)
print(latex_solution)
