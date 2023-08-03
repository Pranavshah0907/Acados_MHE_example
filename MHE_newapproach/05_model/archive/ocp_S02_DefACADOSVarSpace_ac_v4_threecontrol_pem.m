
%% States
import casadi.*

% States 
x = MX.sym('x',options.n_states); 

% Winding / stator temperatur ( K ) 
T_w = x(1);
% Rotor temperature ( K ) 
T_r = x(2);
% Vehicle speed ( m/s ) 
v = x(3);


%% Differential State Variables 

% Differntial States 
dx = MX.sym('dx',options.n_states);

% Winding / stator temperatur change rate ( K/s ) 
dotT_w = dx(1);
% Rotor temperature ( K/s ) 
dotT_r = dx(2);
% Vehicle acceleration ( m/s² ) 
dotv = dx(3);
% % change rate of torque ( Nm/s ) 
% dotM_EM = dx(4);


%% Control inputs 

% Control inputs 
u = MX.sym('u',options.n_controls); 

% Desired torque acceleration output of electrical machine ( rad/s ) 
M_EM_acc = u(1);
% Desired torque braking output of electrical machine ( rad/s ) 
M_EM_brk = u(2);
% Desired torque braking output of friction brake ( rad/s ) SOON To be removed
M_fric_brk = u(3);

%% Parameters
p = MX.sym('p',options.n_parameter); 

% Grade of the road / track NOS ( rad / ° ) -> specification needed
phi = p(1);


