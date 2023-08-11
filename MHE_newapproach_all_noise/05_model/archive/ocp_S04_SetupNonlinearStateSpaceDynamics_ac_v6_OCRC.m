%
% Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
% Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
% Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
% Jonas Koenemann, Yutao Chen, Tobias SchÃ¶ls, Jonas Schlagenhauf, Moritz Diehl
%
% This file is part of acados.
%
% The 2-Clause BSD License
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright notice,
% this list of conditions and the following disclaimer in the documentation
% and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.;

%% Set-up nonlinear state-space model of wind turbine

% Get the power losses of the electrical machine from regressions from the
% test bench results / fields
% Iron, Copper and Friction losses in PMSM ( W )
meanx_mot = 6252;
stdx_mot = 3696;
meany_mot = 67.04;
stdy_mot = 49.49;
x_rpm = (v * FDR * 30) / (pi * r_dyn);
x_scaled =  (x_rpm-meanx_mot)/stdx_mot ;
y_scaled = (abs(M_EM)-meany_mot)/stdy_mot; % y_scaled_fric = (y+abs(z)-meany_mot)/stdy_mot;

p00_c =        1172;  
p10_c =       785.1;  
p01_c =        1868;  
p20_c =       555.3; 
p11_c =       937.4;  
p02_c =         673; 
p21_c =       513.8;  
p12_c =       242.1;
p03_c =        -129;
P_Current = p00_c + p10_c*x_scaled + p01_c*y_scaled + p20_c*x_scaled^2 + p11_c*x_scaled*y_scaled + p02_c*y_scaled^2 + p21_c*(x_scaled^2)*y_scaled + p12_c*x_scaled*y_scaled^2 + p03_c*y_scaled^3;

p00_i =       970.5; 
p10_i =        1203;  
p01_i =       867.6; 
p20_i =       559.9; 
p11_i =       958.9;
p02_i =       200.2;
p21_i =         362;
p12_i =       170.7;
p03_i =      -15.11;
P_Iron = p00_i + p10_i*x_scaled + p01_i*y_scaled + p20_i*x_scaled^2 + p11_i*x_scaled*y_scaled + p02_i*y_scaled^2 + p21_i*(x_scaled^2)*y_scaled + p12_i*x_scaled*y_scaled^2 + p03_i*y_scaled^3;

p00_f =       437.7;
p10_f =         348;
p01_f =       -8.25;
P_Frict = p00_f + p10_f*x_scaled + p01_f*y_scaled;

P_w = P_Current + 0.8 * P_Iron;
% Winding losses in PMSM ( W )
P_r = if_else(M_EM  > 0, 0.2 * P_Iron + P_Frict, ...
    0.2 * P_Iron - P_Frict);
% P_r = 0.2 * P_Iron + P_Frict;
% Rotor losses in PMSM ( W )            


%% Explicit Nonlinear State-Space Model

f_expl = [ ...
(-1/C_w*( (T_w-T_c)/R_wc + (T_w-T_e)/R_we + (T_w-T_r)/R_wr) + (1/C_w)*P_w);
% Winding / stator temperatur change rate ( K/s ) 
(-1/C_r*( (T_r-T_w)/R_wr + (T_r-T_e)/R_re) + (1/C_r)*P_r);
% Rotor temperature ( K/s )
(1/Mv) * ((FDR / r_dyn) * M_EM - 0.5*Cd*Af*dens*v*v - fr*Mv*grav*cos(phi) - Mv*grav*sin(phi) );
% Vehicle acceleration ( m/s² ) 
dt_M_EM;
% torque output rate
];

%% Implicit Nonlinear State-Space Model
f_impl = dx - f_expl;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% output equation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

y = [v; M_EM; dt_M_EM]; % MgenSlack; betaSlack];
% Lagrange Term, integral costs over the full horizon
y_e = [v; M_EM; dt_M_EM];
% y_e = [T_w; T_r; v; M_EM_acc; M_EM_brk; M_fric_brk];
% MAyer term, terminal costs at the end of the horizon



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nonlinear constraints
%
% y = h(x,u)
% yN = hN(xN)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h = [];
h_e = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% linear constraints
%
% y = g(x,u)
% yN = gN(xN)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = [];
g_e = [];