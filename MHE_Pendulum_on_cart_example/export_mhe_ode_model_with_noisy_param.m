function model_mhe = export_mhe_ode_model_with_noisy_param()

import casadi.*

model_name = 'mhe_pendulum';

nx_augmented = 5;
nw = 5;
ny = 4;
nx = 4;

%% system parameters
M = 1;    % mass of the cart [kg]
m = 0.1;  % mass of the ball [kg]
% l = 0.8;  % length of the rod [m] --> Now estimated
g = 9.81; % gravity constant [m/s^2]

%% state symbolics

x1  	= SX.sym('x1');         % horizontal displacement of cart [m]
theta   = SX.sym('theta');      % angle of rod with the vertical [rad]
v1       = SX.sym('v1');          % horizontal velocity of cart [m/s]
dtheta  = SX.sym('dtheta');     % angular velocity of rod [rad/s]

% add parameter l as state
l       = SX.sym('l');          % length of the rod [m]

sym_x = vertcat(x1, theta, v1, dtheta,l);

%% state noise

w_x1      = SX.sym('w_x1');
w_v1      = SX.sym('w_v1');
w_theta   = SX.sym('w_theta');
w_dtheta  = SX.sym('w_dtheta');
w_l       = SX.sym('w_l');

sym_w = vertcat(w_x1, w_theta, w_v1, w_dtheta, w_l);

%% xdot

x1_dot  	= SX.sym('x1_dot');         % horizontal displacement of cart [m]
theta_dot   = SX.sym('theta_dot');      % angle of rod with the vertical [rad]
v1_dot       = SX.sym('v1_dot');          % horizontal velocity of cart [m/s]
dtheta_dot  = SX.sym('dtheta_dot');     % angular velocity of rod [rad/s]
l_dot       = SX.sym('l_dot');          % length of the rod [m]

sym_xdot = vertcat(x1_dot, theta_dot, v1_dot, dtheta_dot, l_dot);

%sym_xdot = SX.sym('xdot', nx+1, 1);

% algebraic variables
sym_z = [];

% parameters <= controls
F       = SX.sym('F');          % horizontal force acting on cart [N]
sym_p   = F;


%% dynamics
%expr_f_expl = vertcat(v, ...
%                      dtheta, ...
%                      (- l*m*sin(theta)*dtheta.^2 + F + g*m*cos(theta)*sin(theta))/(M + m - m*cos(theta).^2), ...
%                      (- l*m*cos(theta)*sin(theta)*dtheta.^2 + F*cos(theta) + g*m*sin(theta) + M*g*sin(theta))/(l*(M + m - m*cos(theta).^2)));
sin_theta = sin(theta);
cos_theta = cos(theta);
denominator = M + m - m*cos_theta.^2;
expr_f_expl = vertcat(v1, ...
                      dtheta, ...
                      (- l*m*sin_theta*dtheta.^2 + F + g*m*cos_theta*sin_theta)/denominator, ...
                      (- l*m*cos_theta*sin_theta*dtheta.^2 + F*cos_theta + g*m*sin_theta + M*g*sin_theta)/(l*denominator), ...
                      0);

% add additive noise 
expr_f_expl = expr_f_expl + sym_w;
expr_f_impl = sym_xdot - expr_f_expl;

cost_expr_y_0  = vertcat(sym_x(1:nx), sym_w, sym_x); % need to be changed
cost_expr_y = vertcat(sym_x(1:nx), sym_w);

%% populate structure

% model.nx = nx;
% model.nu = nu;

% From py
model_mhe.sym_x = sym_x;
model_mhe.sym_xdot = sym_xdot;
model_mhe.sym_u = sym_w;
model_mhe.sym_z = sym_z;
model_mhe.sym_p = sym_p;
model_mhe.name = model_name;
% Till here

model_mhe.cost_expr_y_0 = cost_expr_y_0;
model_mhe.cost_expr_y = cost_expr_y;

model_mhe.expr_f_expl = expr_f_expl;
model_mhe.expr_f_impl = expr_f_impl;

end










