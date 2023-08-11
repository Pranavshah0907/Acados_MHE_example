import casadi.*


%% load trained ANN
% load([pwd '\02_Parameter\LSTM_16HU_v1.mat']); % LSTM,HU and Denormalization parameters
load([pwd '\02_Parameter\LSTM_16HU_v1.mat']); 
% load([pwd '\02_Parameter\LSTM_4HU_v1.mat']); 
% Layer LSTM
InputWeights_lstm = double(net_LSTM.Layers(2, 1).InputWeights);
RecurrentWeights_lstm = double(net_LSTM.Layers(2, 1).RecurrentWeights);
Bias_lstm = double(net_LSTM.Layers(2, 1).Bias);
% Layer FC
Weights_fc2 = double(net_LSTM.Layers(3, 1).Weights);
Bias_fc2 = double(net_LSTM.Layers(3, 1).Bias);

% Normalization parameters
n_em_max = 15000; % [rpm]
T_max = 160; % Celsius degree
T_cool = 60; % Celsius degree
T_range = T_max-T_cool; 


%% Deadtime, Options Solver and MPC Constants
options = struct;
% options.solver              = 'ipopt'; % solver
options.n_states            = 3 + 2 * numHiddenUnits;
options.n_states_bound      = options.n_states - 2 * numHiddenUnits;
options.n_outputs           = 4;
options.n_controls          = 35; %3; % torque
options.n_parameter         = 5; %2; % with v_ref
options.n_slacks            = 4;
options.n_gzus              = 3; % zus G Constraints U2*U1, U3*U1
options.n_eqg               = 3; % number of additional equality constraints in g (U1 * U2 = 0)
options.M                   = 40; % number of control intervals
options.P                   = 15; %40; % number of prediction intervals, P >= N
options.Ts                  = 0.10; % Time step lenght [s]
options.T                   = options.P * options.Ts; % Final time horizon length in seconds [s]
options.maxIter             = 30;  % Maximum iterations for solver (nlp steps)
options.sqp_steps           = 100; %5; % spq steps
options.nRK                 = 4; 

deadtime_T = 0;                            % Deadtime on S-Func for EM Temperatures (delay)% s
delta_k_deadtime = round(deadtime_T / options.Ts); % -

% struct for lstm simulation

% Par_LSTM_Sim = struct ();
% Par_LSTM_Sim.InputWeights_lstm = InputWeights_lstm;
% Par_LSTM_Sim.RecurrentWeights_lstm = RecurrentWeights_lstm;
% Par_LSTM_Sim.Bias_lstm = Bias_lstm;
% Par_LSTM_Sim.Weights_fc2 = Weights_fc2;
% Par_LSTM_Sim.Bias_fc2 = Bias_fc2;
% Par_LSTM_Sim.NumHiddenUnits = net_LSTM.Layers(2, 1).NumHiddenUnits;
% Par_LSTM_Sim.Ts = options.Ts;
% numHiddenUnits = Par_LSTM_Sim.numHiddenUnits;
% InputWeights_lstm = Par_LSTM_Sim.InputWeights_lstm;
% RecurrentWeights_lstm = Par_LSTM_Sim.RecurrentWeights_lstm;
% Bias_lstm = Par_LSTM_Sim.Bias_lstm;
% Weights_fc2 = Par_LSTM_Sim.Weights_fc2;
% Bias_fc2 = Par_LSTM_Sim.Bias_fc2;
% Ts = Par_LSTM_Sim.Ts;

%% Constraints
% State variables
% T_w_max = 273.15 + 155; T_w_min = 0; % [K]
% T_r_max = 273.15 + 155; T_r_min = 0;
T_w_max = 155; T_w_min = 0; % [K]
T_r_max = 155; T_r_min = 0;
v_max = 37; v_min = 0; % [m/s], defined by transmission

% Manipulated variables load torque and rev LUT in future
M_EM_acc_max = 160.36; M_EM_acc_min = 0; %[Nm]
M_EM_brk_max = 0; M_EM_brk_min = -160.36; %[Nm]
M_fric_brk_max = 0; M_fric_brk_min = -torque_brake; %[Nm]
dt_M_EM_acc_min = -500; dt_M_EM_acc_max = -dt_M_EM_acc_min; %min dt accel decel IPG = 4s, from excel [Nm/s]

% n_mot_max_red = n_mot_max(38:end);
% M_mot_max_red = M_mot_max(38:end);
% plot(n_mot_max_red, M_mot_max_red)

% P_EM_min = -torque_brake * 2 * pi * max(n_mot_max);
P_EM_min = M_gen_max  * pi/30  .* n_gen_max;
P_EM_min = min(P_EM_min);
P_EM_max = M_mot_max  * pi/30  .* n_mot_max ;
P_EM_max = max(P_EM_max);
% div_P_n = max(P_EM_max) /
% plot(n_mot_max,P_EM_max)

%% Reference Values and Initial Values
mpc_init_EM_acc = 1;
mpc_init_EM_brk = 0;
mpc_init_fric_brk = 0;
 
% T_w_ref = 273.15 + 80; T_r_ref = 273.15 + 80; % [K] Ref 
T_w_ref = 80; T_r_ref = 80; % [°C] Ref
v_ref = 133.5 / 3.6; % [m/s] Ref - km/h / 3.6 todo check

M_EM_acc_ref = 0; M_EM_brk_ref = 0; M_fric_brk_ref = 0; %[Nm]

% X initial states
v_0 = 1;
n_0  = v_0 * FDR * 60;     % speed for which controller is set up [rpm], 
% v_0 = (n_0 *2*pi*r_dyn) / (FDR*60); % initial guess for speed % [m/s]

% T_w_0 = 273.15 + 60; % initial guess for winding temperature % [K]
% T_r_0 = 273.15 + 60; % initial guess for winding temperature % [K]
T_w_0 = 60; % initial guess for winding temperature % [°C]
T_r_0 = 60; % initial guess for winding temperature % [°C]
phi = 0; % Slope []

% U initial states
M_EM_acc_0 = M_EM_acc_max; M_EM_brk_0 = 0; M_fric_brk_0 = 0; %[Nm]
ht0 = [zeros(numHiddenUnits,1)];
ct0 = [zeros(numHiddenUnits,1)];
mv_0 = [M_EM_acc_0, M_EM_brk_0, M_fric_brk_0];
options.x0 = [T_w_0; T_r_0; v_0; ht0; ct0];  % Todo Initial condition of system

x_ref = [T_w_ref; T_r_ref; v_ref]; % todo for what
u_ref = [M_EM_acc_ref;M_EM_brk_ref;M_fric_brk_ref]; % [0;0;0]


%% Scale signals
% % Set scale factors because plant input and output signals have different
% % orders of magnitude
% x_scale = [1 ; 1; 1];
% u_scale = [1 ; 1; 1];
% u_rate_scale = [1 ; 1; 1];
x_scale = [T_w_max; T_r_max; v_max]; %K,K,m/s%
% p_scale = 9.3993; %max(grade_track) ; %* pi / 180; % [T_c; T_e]   
u_scale = [max(M_mot_max); abs(min(M_gen_max));torque_brake]; % [M_em_acc;M_em_brake;M_fric_brake]
urate_scale = [(max(M_mot_max) *  0.5); (abs(min(M_gen_max)) * 0.5) ; (torque_brake * 0.25)]; % [M_em_acc;M_em_brake;M_fric_brake]
% objc_scale = norm(x_scale) + norm(u_scale) + norm(urate_scale);
% P_w_max = 9116; P_r_max= 1684;
% x_scale_lb =[T_w_min, T_r_min, v_min];
% x_scale_ub =[T_w_max, T_r_max, v_max];
% u_scale_lb = [M_EM_acc_min, M_EM_brk_min, M_fric_brk_min];
% u_scale_ub = [M_EM_acc_max, M_EM_brk_max, M_fric_brk_max];

%% Weights

% if options.Ts == 0.1
% weights.aggro = 1;
% Q = zeros(options.n_states,options.n_states); % weighing matrices T_W, T_r, v (states)
% %     Q = MX(3,3); % T_W, T_r, v (states)
%  % y = [v_1; M_EM_acc; M_EM_brk; M_fric_brk];
% Q(1,1) = (1/x_scale(1)^2) * weights.aggro * 0.01; % T_w
% Q(2,2) = (1/x_scale(2)^2) * weights.aggro * 0.01; % T_r
% % Q(3,3) = (1/x_scale(3)^2) * weights.aggro * 2; % Focus on following v_ref striclty
% Q(3,3) = (1/x_scale(3)^2) * weights.aggro * 1; % v
% % Q(1,1) = weights.aggro * 0.1;
% % Q(2,2) = weights.aggro * 0.1;
% % Q(3,3) = weights.aggro * 1;
% % weights.OV = [full(Q(1,1));full(Q(2,2));full(Q(3,3))];
% weights.OV = diag(Q)'; % T_W, T_r, v
% 
% R = zeros(options.n_controls,options.n_controls);  %M_em_acc M_em_brake M_fric_brake (controls) todo   
% R(1,1) = (1/u_scale(1)^2) * 0.01; %M_em_acc
% R(2,2) = (1/u_scale(2)^2) * 0.01; % M_em_brk
% R(3,3) = (1/u_scale(3)^2) * 0.01; % M_fric_brk
% % R(1,1) = 5e-3;
% weights.MV = diag(R)';  %M_em_acc M_em_brake M_fric_brake   
% 
% U = zeros(options.n_controls,options.n_controls);  %delta_M_em_acc delta_M_em_brake delta_M_fric_brake (controls)   
% weights.alpha = 1; % 3000
% U(1,1) = (1/dt_M_EM_acc_min^2) * weights.alpha * 5e-2;%5e-2; %delta_M_em
% U(2,2) = (1/dt_M_EM_acc_min^2) * weights.alpha * 5e-2;
% U(3,3) = (1/dt_M_EM_acc_min^2) * weights.alpha * 5e-2;
% % U(1,1) = 5e-3; %delta_M_em
% weights.MVrate = diag(U)';  %M_em_acc M_em_brake M_fric_brake   
% 
% % Slack Variables         
% S = zeros(options.n_slacks,options.n_slacks); 
% weights.rho = 1;
% S(1,1) = (1/x_scale(1)^2) * weights.rho * 1; % Slack on T_w constraints
% S(2,2) = (1/x_scale(2)^2) * weights.rho * 0.01; % Slack on T_r constraints
% S(3,3) = (1/x_scale(3)^2) * weights.rho * 0.01; % Slack on v constraints
% S(4,4) = (1/u_scale(1)^2) * weights.rho * 0.01; % Slack on u constraints
% S(5,5) = (1/dt_M_EM_acc_min^2) * weights.rho * 0.01; % Slack on dt constraints
% weights.EPS = diag(S)';
% end
% 
% if options.Ts < 0.1
% weights.aggro = 1;
% Q = zeros(options.n_states,options.n_states); % T_W, T_r, v (states)
% %     Q = MX(3,3); % T_W, T_r, v (states)
% Q(1,1) = (1/x_scale(1)^2) * weights.aggro * 0.01;
% Q(2,2) = (1/x_scale(2)^2) * weights.aggro * 0.01;
% Q(3,3) = (1/x_scale(3)^2) * weights.aggro * 1; % weighing matrices (states) % Focus on following v_ref striclty
% % Q(1,1) = weights.aggro * 0.1;
% % Q(2,2) = weights.aggro * 0.1;
% % Q(3,3) = weights.aggro * 1;
% % weights.OV = [full(Q(1,1));full(Q(2,2));full(Q(3,3))];
% weights.OV = diag(Q)'; % T_W, T_r, v
% 
% R = zeros(options.n_controls,options.n_controls);  %M_em_acc M_em_brake M_fric_brake (controls)   
% R(1,1) = (1/u_scale(1)^2) * 0.1; %M_em_acc,u1
% R(2,2) = (1/u_scale(2)^2) * 0.05; %u2
% R(3,3) = (1/u_scale(3)^2) * 0.05; %u3
% % R(1,1) = 5e-3;
% weights.MV = diag(R)';  %M_em_acc M_em_brake M_fric_brake   
% 
% U = zeros(options.n_controls,options.n_controls);  %delta_M_em_acc delta_M_em_brake delta_M_fric_brake (controls)   
% weights.alpha = 1; % 3000
% U(1,1) = (1/dt_M_EM_acc_min^2) * weights.alpha * 5e-2; %delta_M_em_acc
% U(2,2) = (1/dt_M_EM_acc_min^2) * weights.alpha * 5e-2; %not penalized
% U(3,3) = (1/dt_M_EM_acc_min^2) * weights.alpha * 5e-2; %not penalized
% % U(1,1) = 5e-3; %delta_M_em
% weights.MVrate = diag(U)';  %M_em_acc M_em_brake M_fric_brake   
% 
% % Slack Variables         
% S = zeros(options.n_slacks,options.n_slacks); 
% weights.rho = 1;
% S(1,1) = (1/x_scale(1)^2) * weights.rho * 1; % Slack on T_w constraints
% S(2,2) = (1/x_scale(2)^2) * weights.rho * 0.01; % Slack on T_r constraints
% S(3,3) = (1/x_scale(3)^2) * weights.rho * 0.01; % Slack on v constraints
% S(4,4) = (1/u_scale(1)^2) * weights.rho * 0.01; % Slack on u constraints
% S(5,5) = (1/dt_M_EM_acc_min^2) * weights.rho * 0.01; % Slack on dt constraints
% weights.EPS = diag(S)';
% end

% 
% if options.Ts > 0.1
% weights.aggro = 1;
% Q = zeros(options.n_states,options.n_states); % T_W, T_r, v (states)
% %     Q = MX(3,3); % T_W, T_r, v (states)
% Q(1,1) = (1/x_scale(1)^2) * weights.aggro * 0.01;
% Q(2,2) = (1/x_scale(2)^2) * weights.aggro * 0.01;
% Q(3,3) = (1/x_scale(3)^2) * weights.aggro * 0.5; % weighing matrices (states) % Focus on following v_ref striclty
% % Q(1,1) = weights.aggro * 0.1;
% % Q(2,2) = weights.aggro * 0.1;
% % Q(3,3) = weights.aggro * 1;
% % weights.OV = [full(Q(1,1));full(Q(2,2));full(Q(3,3))];
% weights.OV = diag(Q)'; % T_W, T_r, v
% 
% R = zeros(options.n_controls,options.n_controls);  %M_em_acc M_em_brake M_fric_brake (controls)   
% R(1,1) = (1/u_scale(1)^2) * 1e-1; %M_em_
% R(2,2) = (1/u_scale(2)^2) * 1e-2;
% R(3,3) = (1/u_scale(3)^2) * 10e-2;
% % R(1,1) = 5e-3;
% weights.MV = diag(R)';  %M_em_acc M_em_brake M_fric_brake   
% 
% U = zeros(options.n_controls,options.n_controls);  %delta_M_em_acc delta_M_em_brake delta_M_fric_brake (controls)   
% weights.alpha = 1; % 3000
% U(1,1) = (1/dt_M_EM_acc_min^2) * weights.alpha * 5e-2; %delta_M_em
% U(2,2) = (1/dt_M_EM_acc_min^2) * weights.alpha * 5e-2;
% U(3,3) = (1/dt_M_EM_acc_min^2) * weights.alpha * 5e-2;
% % U(1,1) = 5e-3; %delta_M_em
% weights.MVrate = diag(U)';  %M_em_acc M_em_brake M_fric_brake   
% 
% % Slack Variables         
% S = zeros(options.n_slacks,options.n_slacks); 
% weights.rho = 1;
% S(1,1) = (1/x_scale(1)^2) * weights.rho * 1; % Slack on T_w constraints
% S(2,2) = (1/x_scale(2)^2) * weights.rho * 0.01; % Slack on T_r constraints
% S(3,3) = (1/x_scale(3)^2) * weights.rho * 0.01; % Slack on v constraints
% S(4,4) = (1/u_scale(1)^2) * weights.rho * 0.01; % Slack on u constraints
% S(5,5) = (1/dt_M_EM_acc_min^2) * weights.rho * 0.01; % Slack on dt constraints
% weights.EPS = diag(S)';  
% end