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
meanx_mot = 5288; stdx_mot = 3429; meany_mot = 67.02; stdy_mot = 54.59;
x_rpm = (v * FDR * 30) / (pi * r_dyn);
x_scaled =  (x_rpm-meanx_mot)/stdx_mot ;
y_scaled = (M_EM-meany_mot)/stdy_mot; % y_scaled_fric = (y+abs(z)-meany_mot)/stdy_mot;

p00 =        1719  ;
p10 =       700.6  ;
p01 =        1932  ;
p20 =      -72.98  ;
p11 =       135.7  ;
p02 =       5.422  ;
p21 =      -220.3  ;
p12 =      -415.1  ;
p03 =      -211.7  ;
P_Current = p00 + p10*x_scaled + p01*y_scaled + p20*x_scaled^2 + p11*x_scaled*y_scaled + p02*y_scaled^2 + p21*(x_scaled^2)*y_scaled + p12*x_scaled*y_scaled^2 + p03*y_scaled^3;

p00 =        1339  ;
p01 =       992.9  ;
p20 =       14.01  ;
p11 =       347.3  ;
p02 =      -140.3  ;
p21 =        -154  ;
p12 =      -462.2  ;
p03 =       95.78  ;
P_Iron = p00 + p10*x_scaled + p01*y_scaled + p20*x_scaled^2 + p11*x_scaled*y_scaled + p02*y_scaled^2 + p21*(x_scaled^2)*y_scaled + p12*x_scaled*y_scaled^2 + p03*y_scaled^3;

p00 =       362.1; %  (357.5, 366.8) %  Linear model Poly11 with Robust LAR:
p10 =       262.4; %  (257.7, 267.1)
p01 =       9.394; %  (4.735, 14.05)
P_Frict = p00 + p10*x_scaled + p01*y_scaled;

P_w = P_Current + 0.8 * P_Iron;
% Winding losses in PMSM ( W )
P_r = if_else(M_EM > 0, 0.2 * P_Iron + P_Frict, ...
    0.2 * P_Iron - P_Frict);
% P_r = 0.2 * P_Iron + P_Frict;
% Rotor losses in PMSM ( W )            


%% Explicit Nonlinear State-Space Model

f_expl = [ ...
-1/C_w*( (T_w-T_c)/R_wc + (T_w-T_e)/R_we + (T_w-T_r)/R_wr) + (1/C_w)*P_w;
% Winding / stator temperatur change rate ( K/s ) 
-1/C_r*( (T_r-T_w)/R_wr + (T_r-T_e)/R_re) + (1/C_r)*P_r;
% Rotor temperature ( K/s )
1/Mv * ((FDR / r_dyn * M_EM) - 0.5*Cd*Af*dens*v*v - fr*Mv*grav*cos(phi) - Mv*grav*sin(phi));
% Vehicle acceleration ( m/s² ) 
];

%% Implicit Nonlinear State-Space Model
f_impl = dx - f_expl;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% output equation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

y = [T_w; T_r; v; M_EM];% ; MgenSlack; betaSlack];
% Lagrange Term, integral costs over the full horizon
y_e = [T_w; T_r; v];
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

