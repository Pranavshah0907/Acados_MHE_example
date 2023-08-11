%% States
import casadi.*

% States 
x = SX.sym('x',options.n_states); 

% Winding / stator temperatur ( K ) 
T_w = x(1);
% Rotor temperature ( K ) 
T_r = x(2);
% Vehicle speed ( m/s ) 
v = x(3);
% PMSM Acc Torque ( Nm )
M_EM_acc = x(4);
% PMSM Dec Torque ( Nm )
M_EM_brk = x(5);
% Friction Brake Dec Torque ( Nm )
M_fric_brk = x(6);


%% Differential State Variables 

% Differntial States 
dx = SX.sym('dx',options.n_states);

% Winding / stator temperatur change rate ( K/s ) 
dotT_w = dx(1);
% Rotor temperature ( K/s ) 
dotT_r = dx(2);
% Vehicle acceleration ( m/s² ) 
dotv = dx(3);
% % change rate of torque ( Nm/s )
dotM_EM_acc = dx(4);
% change rate of PMSM Dec Torque ( Nm/s )
dotM_EM_brk = x(5);
% change rate of Friction Brake Dec Torque ( Nm/s )
dotM_fric_brk = x(6);


%% Control inputs 

% Control inputs 
u = SX.sym('u',options.n_controls); 

% Desired torque acceleration output of electrical machine
dt_M_EM_acc = u(1);
% Desired torque braking output of electrical machine 
dt_M_EM_brk = u(2);
% Desired torque braking output of friction brake
dt_M_fric_brk = u(3);

%% Parameters
p = SX.sym('p',options.n_parameter); 

% Grade of the road / track NOS ( rad / ° ) -> specification needed
phi = p(1);


