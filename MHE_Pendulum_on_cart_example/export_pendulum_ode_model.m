function model = export_pendulum_ode_model()

import casadi.*

model_name = 'pendulum_ode';

%% system dimensions
nx = 4;
nu = 1;

%% system parameters
M = 1;    % mass of the cart [kg]
m = 0.1;  % mass of the ball [kg]
l = 0.8;  % length of the rod [m]
g = 9.81; % gravity constant [m/s^2]

%% named symbolic variables
x1 = SX.sym('x1');         % horizontal displacement of cart [m]
theta = SX.sym('theta'); % angle of rod with the vertical [rad]
v = SX.sym('v');         % horizontal velocity of cart [m/s]
dtheta = SX.sym('dtheta'); % angular velocity of rod [rad/s]
F = SX.sym('F');         % horizontal force acting on cart [N]

%% (unnamed) symbolic variables
sym_x = vertcat(x1, theta, v, dtheta);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = F;

sym_p = []; % PS 

%% dynamics
%expr_f_expl = vertcat(v, ...
%                      dtheta, ...
%                      (- l*m*sin(theta)*dtheta.^2 + F + g*m*cos(theta)*sin(theta))/(M + m - m*cos(theta).^2), ...
%                      (- l*m*cos(theta)*sin(theta)*dtheta.^2 + F*cos(theta) + g*m*sin(theta) + M*g*sin(theta))/(l*(M + m - m*cos(theta).^2)));
sin_theta = sin(theta);
cos_theta = cos(theta);
denominator = M + m - m*cos_theta.^2;
expr_f_expl = vertcat(v, ...
                      dtheta, ...
                      (- l*m*sin_theta*dtheta.^2 + F + g*m*cos_theta*sin_theta)/denominator, ...
                      (- l*m*cos_theta*sin_theta*dtheta.^2 + F*cos_theta + g*m*sin_theta + M*g*sin_theta)/(l*denominator));
expr_f_impl = sym_xdot - expr_f_expl;

%% constraints
% expr_h = sym_u;

%% cost
Q_ocp = diag([1e3, 1e3, 1e-2, 1e-2]);
R_ocp = 1e-2;

% expr_ext_cost_e = sym_x'* W_x * sym_x;
% expr_ext_cost = expr_ext_cost_e + sym_u' * W_u * sym_u;
% nonlinear least sqares
cost_expr_y  = vertcat(sym_x, sym_u);
W = blkdiag(Q_ocp, R_ocp);


%% populate structure
model.nx = nx;
model.nu = nu;

% Below same as py
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.name = model_name;
model.sym_p = sym_p;

% model.expr_h = expr_h;
% model.expr_ext_cost = expr_ext_cost;
% model.expr_ext_cost_e = expr_ext_cost_e;
% 
model.cost_expr_y = cost_expr_y;
model.cost_expr_y_e = sym_x;

model.W = W;
model.W_e = Q_ocp;

end
