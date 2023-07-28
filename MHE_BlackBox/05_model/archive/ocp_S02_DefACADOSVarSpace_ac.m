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
%

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


