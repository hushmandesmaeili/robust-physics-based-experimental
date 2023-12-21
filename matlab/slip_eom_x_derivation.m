% Define symbolic constants, functions, and initial conditions
syms t T p_T p_0 m g h real
syms x(t) p_x(t)
syms x_0 xdot_0 real

% Define the variable p_x(t)
p_x = (t/T)*(p_T - p_0) + p_0;

% Define the second-order differential equation
eq = m*diff(x, t, 2) == (m*g*(x - p_x))/h;

% Solve the differential equation symbolically
% sol = dsolve(eq, x(0) == x_0, Dx(0) == xdot_0);
% Solve the differential equation with initial conditions
sol = dsolve(eq, x(0) == x_0, subs(diff(x, t), t, 0) == xdot_0);

% Display the solution
disp('Symbolic Solution:');
latexString = latex(sol);
disp(latexString);

% Display the solution using a formula
figure;
annotation('textbox', [0.1 0.1 0.8 0.8], 'String', ['$$' latexString '$$'], ...
    'Interpreter', 'latex', 'FontSize', 14, 'FitBoxToText', 'on');
axis off;

% Save LaTeX-formatted solution to a file
fid = fopen('slip_eom_x_latex_solution.tex', 'w');
fprintf(fid, '%s', latex(sol));
fclose(fid);

% % You can substitute specific values for parameters if needed
% % For example, substitute T = 1, p_T = 2, p_0 = 1
% sol_numeric = subs(sol, {T, p_T, p_0}, {1, 2, 1});
% 
% % Display the numeric solution
% disp('Numeric Solution (substituting T=1, p_T=2, p_0=1):');
% disp(sol_numeric);
