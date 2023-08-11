%% BOF

timestep = options.Ts; 
% %% extract states

%% Hidden Layer 1 FC
% ut is inputs for the LSTM: rotational speed, torque, T_w, T_r
% transformation speed and rotational speed: x_rpm = (v * FDR * 30) / (pi * r_dyn);
% inputs_LSTM = [(v * FDR * 30) / (pi * r_dyn); (M_EM_acc + M_EM_brk); T_w; T_r];
% n_em_max = 15000; % [rpm]
% T_max = 160; % Celsius degree
% T_cool = 60; % Celsius degree
% T_range = T_max-T_cool; % Todo Cut to init later
inputs_FFNN = [(v * FDR * 30) / (pi * r_dyn * n_em_max); (M_EM_acc + M_EM_brk) / M_EM_acc_max;...
    (T_w - T_cool) / T_range; (T_r - T_cool) / T_range]; % Normalized

Z_fc1 = Weights_fc1*inputs_FFNN + Bias_fc1;

%% Hidden Layer 2 FC2
Z_fc2 = Weights_fc2*ht + Bias_fc2; % fully connected 7 Output: (dTw/dt, dTr/dt) -> 1 degree / 0.1 s
% integration step of dTw, dTr
T_w_1 = timestep * (Z_fc2(1)' * (YMax - YMin) + YMin) + T_w; % Denormalized
T_r_1 = timestep * (Z_fc2(2)' * (YMax - YMin) + YMin) + T_r; %

% T_w_1 = timestep * (Z_fc2(1)') + T_w; % DENORMALIZE here TODO
% T_r_1 = timestep * (Z_fc2(2)') + T_r; %


%% vehicle dynmics
% discretize the explicit ODE of v_dot
% v_dot = f ( x, u, p)     -> v_dot!!!!! =! v_next
% v_dot ~ v ²
% -> discretize by integration to v_next = f (x,u,p)
disp(v)
v_dot = 1/Mv * ((FDR / r_dyn * (M_EM_acc + M_EM_brk + M_fric_brk)) - 0.5*Cd*Af*dens*v*v - fr*Mv*grav*cos(phi) - Mv*grav*sin(phi));
ode = casadi.Function('ode', {v, M_EM_acc, M_EM_brk, M_fric_brk, phi}, {v_dot}, ...
    {'v', 'M_EM_acc', 'M_EM_brk', 'M_fric_brk', 'phi'},{'v_dot'});
disp(ode)
% f = Function('f',{x,y},...
%       {x,sin(y)*x},...
%       {'x','y'},{'r','q'});
% disp(f)

% Set up explicit Euler first order
% k1 = ode(v, M_EM_acc, M_EM_brk, M_fric_brk, phi);
% v_1 = v + timestep * k1;

% Set up explicit runge kutta 4 order, RK4
k1 = ode(v, M_EM_acc, M_EM_brk, M_fric_brk, phi);
k2 = ode(v + timestep/2*k1, M_EM_acc, M_EM_brk, M_fric_brk, phi);
k3 = ode(v + timestep/2*k2, M_EM_acc, M_EM_brk, M_fric_brk, phi);
k4 = ode(v + timestep*k3, M_EM_acc, M_EM_brk, M_fric_brk, phi);
v_1 = v + timestep/6 * (k1 + 2*k2 + 2*k3 + k4);

%% Discrete Model
expr_phi = [T_w_1; T_r_1; v_1]; % next states / x_(k+1)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% output equation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% y = [v; M_EM_acc; dt_M_EM_acc; M_EM_brk; M_fric_brk];
y = [v_1; M_EM_acc; M_EM_brk; M_fric_brk];
% Lagrange Term, integral costs over the full horizon
y_e = [v_1; M_EM_acc; M_EM_brk; M_fric_brk];
% y_e = [v; M_EM_acc; dt_M_EM_acc; M_EM_brk; M_fric_brk];
% y_e = [T_w; T_r; v; M_EM_acc; M_EM_brk; M_fric_brk];
% MAyer term, terminal costs at the end of the horizon



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nonlinear constraints
%
% y = h(x,u)
% yN = hN(xN)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% h = [];
h = [M_EM_acc * M_EM_brk; M_EM_acc * M_fric_brk; (abs(M_EM_brk)+M_EM_acc)*v_1*FDR/r_dyn];
% h_e = [];
h_e = [M_EM_acc * M_EM_brk; M_EM_acc * M_fric_brk; (abs(M_EM_brk)+M_EM_acc)*v_1*FDR/r_dyn];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% linear constraints
%
% y = g(x,u)
% yN = gN(xN)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = [];
g_e = [];