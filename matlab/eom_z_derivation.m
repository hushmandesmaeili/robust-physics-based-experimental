syms m k g r_T r_0 real
syms z(t) r(t)
syms z_0 zdot_0

% Define the variable r(t)
r = (t/T)*(r_T - r_0) + r_0;

% Define the second-order differential equation
eq = m*diff(z, t, 2) == k*(r - z) - m*g
% Display the solution using a formula
figure;
annotation('textbox', [0.1 0.1 0.8 0.8], 'String', ['$$' latex(eq) '$$'], ...
    'Interpreter', 'latex', 'FontSize', 14, 'FitBoxToText', 'on');
axis off;

% Solve the differential equation with initial conditions
sol = dsolve(eq);
% Solve the differential equation with initial conditions
% sol = dsolve(eq, z(0) == z_0, subs(diff(z, t), t, 0) == zdot_0);

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
fid = fopen('slip_eom_z_latex_solution.tex', 'w');
fprintf(fid, '%s', latex(sol));
fclose(fid);