import casadi.*

% States % TODO: change names like ht1 
% Winding / stator temperatur (°C) 
T_w = SX.sym('T_w', 1);
% Rotor temperature (°C) 
T_r = SX.sym('T_w', 1);
% Vehicle speed ( m/s ) 
v = SX.sym('v', 1);
% % PMSM Acc Torque ( Nm )
% M_EM_acc = x(4);
% % PMSM Dec Torque ( Nm )
% M_EM_brk = x(5);
% % Friction Brake Dec Torque ( Nm )
% M_fric_brk = x(6);

ht = SX.sym('ht',numHiddenUnits); % HiddenhHiddeH
ct = SX.sym('ct',numHiddenUnits); % Cell states

% x = vertcat(T_w, T_r, v, M_EM_acc, M_EM_brk, M_Fric_brk, ht1, ct1); % states, add all statee
x = vertcat(T_w, T_r, v, ht, ct); % states, add all states 35

%% differential states
v_dot = SX.sym('v_dot', 1); % acceleration, this is discretized in ocp:S04

%% algebraic variables
i_t = SX.sym('i_t',numHiddenUnits); % Input gate (sigmoid function)
f_t = SX.sym('f_t',numHiddenUnits); % Forget gate
g_t = SX.sym('g_t',numHiddenUnits); % Cell candidate
o_t = SX.sym('o_t',numHiddenUnits); % Output gate

%% adding state noises - new

w_Tw = SX.sym('w_Tw');
w_Tr = SX.sym('w_Tr');
% w_v = SX.sym('w_v');
% w_ht = SX.sym('w_ht');
% w_ct = SX.sym('w_ct');
%zeros_for_HU = zeros(2*numHiddenUnits,1);
zeros_for_HU = SX.sym('hu',numHiddenUnits*2+1);
sym_w = vertcat(w_Tw, w_Tr, zeros_for_HU); %35

%% Controls as parameters - new

% Desired torque acceleration output of electrical machine ( rad/s ) 
M_EM_acc = SX.sym('M_EM_acc', 1);
% Desired torque braking output of electrical machine ( rad/s ) 
M_EM_brk = SX.sym('M_EM_brk', 1);
% Desired torque braking output of friction brake ( rad/s )
M_fric_brk = SX.sym('M_fric_brk', 1);
% % Desired torque acceleration output of electrical machine
% dt_M_EM_acc = u(1);
% % Desired torque braking output of electrical machine 
% dt_M_EM_brk = u(2);
% % Desired torque braking output of friction brake
% dt_M_fric_brk = u(3);

%% Old Parameters
% Grade of the road / track 
phi = SX.sym('phi', 1);
v_r = SX.sym('v_r',1 );
p = vertcat(M_EM_acc, M_EM_brk, M_fric_brk, phi, v_r); % phi (+ T_w_0 + T_r_0)


% T_w_0 = p(2);
% T_r_0 = p(3);