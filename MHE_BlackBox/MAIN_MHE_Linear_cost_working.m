%% || Shree ||

clear; clc; close all;
clear mex;

[projectRootDir,~,~] = fileparts(mfilename('fullpath'));
idcs = strfind(projectRootDir,'\'); % find \ string
userpath = projectRootDir(1:idcs(end)-1); % get parent directory

pathstr = '\MHE_BlackBox';
cd([userpath, pathstr]);

% check that env.sh has been run / acados stuff
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	disp('env.sh has not been sourced! Before executing this example, we run: source env.sh');
    cd ([userpath '\acados\examples\acados_matlab_octave']);
    acados_env_variables_windows;
    disp('env.sh has been sourced! Continue with NMPC');
    cd([userpath, pathstr]);
end

%% Initialization


addpath(genpath('..\MHE_BlackBox'))
addpath(genpath('..\acados\external\casadi-matlab'))
load("input_data.mat")
init_modvehicle_mpc_IPG;    %Init Vehicle parameters
init_MPC_Constants_slacks_LSTM;     % Init constants MPC and Options P,T, etc % _v2 with Constraints

model_name = 'LSTM_v21_MHE'; 
model = ocp_MHE_model_NMPC_v8_LSTM; 

%% arguments for solver

compile_interface = 'auto';
codgen_model = 'true';

nlp_solver = 'sqp';
nlp_solver_exact_hessian = 'false';
regularize_method = 'no_regularize';
nlp_solver_max_iter = options.sqp_steps;

% investigate all the below von
nlp_solver_ext_qp_res = 0;
% qp_solver = 'full_condensing_hpipm'; %qp_solver = 'full_condensing_qpoases'; %qp_solver = 'partial_condensing_osqp'; %qp_solver = 'partial_condensing_hpmpc';
% qp_solver = 'partial_condensing_hpipm'; 
qp_solver = 'partial_condensing_hpipm'; % ToDo: Investigate
qp_solver_cond_N = 5;
qp_solver_cond_ric_alg = 0;
qp_solver_ric_alg = 0;
qp_solver_warm_start = 1;
qp_solver_max_iter = options.maxIter;
qp_solver_print_level = 1;
% param_scheme = 'multiple_shooting'; % 'single_shooting'
% sim_method = 'erk';
% sim_method = 'irk';
sim_method = 'discrete';
% bis

N = options.P;
cost_type = 'linear_ls';

%% Dims to set up the cost

dims.Ts = options.Ts; % sampling time
dims.T = N*dims.Ts; % horizon length time [s]
dims.nx = model.nx; % 4
dims.nu = model.nu; % 3
% dims.ny = length(model.expr_y); % number of outputs in lagrange term y
dims.ny = 2; % T_w and T_r
%dims.ny_e = length(model.expr_y_e); % number of outputs in mayer term y_e
dims.nbx = dims.nx; % nx -  2* numHiddenUnits; % bounds
%dims.nbx_e = dims.nbx; %
dims.nsbx = 0; %nx;
dims.nsbx_e = 0; %nx; %
dims.nbu = dims.nu;
dims.nsbu = 0;
dims.ng = length(model.expr_g);
dims.ng_e = length(model.expr_g_e);
dims.nsg = dims.ng;
dims.nsg_e = dims.ng_e;
dims.nh = length(model.expr_h); % +1
dims.nh_e = length(model.expr_h_e); % +1
dims.nsh = dims.nh; % +1
dims.nsh_e = 0; %nh_e;
dims.ns = dims.nsh + dims.nsbx + dims.nsbu + dims.ng; % +1
dims.ns_e = dims.nsh_e + dims.nsbx_e + dims.ng_e; % +1
dims.np = model.np; % +1

%% Weights for cost

% Q = 1;
% R = 1;

%% Cost

nout = dims.ny + dims.nu;
nout_0 = dims.ny + dims.nu + dims.nx;

% state-to-output matrix in lagrange term
Vx = zeros(nout, dims.nx);
Vx(1, 1) = 1.0; % T_w
Vx(2, 2) = 1.0; % T_r 

% input-to-output matrix in lagrange term
Vu = zeros(nout, dims.nu);
Vu(dims.ny+1:dims.ny+dims.nu, 1:dims.nu) = eye(dims.nu);

% PS

yref = zeros(nout, 1);
yref_0 = zeros(nout_0, 1);

Vx_0 = zeros(nout_0, dims.nx);
Vx_0(1:dims.ny, :) = eye(dims.ny, dims.nx);
Vx_0(dims.ny+dims.nu+1:end, :) = eye(dims.nx);

Vu_0 = zeros(nout_0, dims.nu); 
Vu_0(dims.ny+1:dims.ny+dims.nu, :) = eye(dims.nu);
% PS end 

% Vx_0; Vu_0; y_ref; y_ref_0 --> need to be defined before the
% simualtion
W = [10 0 0 0 0; % y1
    0 10 0 0 0;  % y2
    0 0 1 0 0;  % u1
    0 0 0 1 0;  % u2
    0 0 0 0 1]; % u3
% W(1, 1) =  Q(3,3); % v
% W(2, 2) =  R(1,1); % u1
% W(3, 3) =  R(2,2); % u2
% W(4, 4) =  R(3,3); % u3

W_0 = eye(nout_0, nout_0);

% The W and W_0 are the matrices of Q and R which needs to be defined.



%% Constraints

% % % state bounds terminal mayer term
% Jbx_e = eye(dims.nbx, dims.nx);
% lbx_e = [T_w_min; T_r_min; v_min; -1 * ones(2*numHiddenUnits,1)];
% ubx_e = [T_w_max; T_r_max; v_max; ones(2*numHiddenUnits,1)];

% state bounds
Jbx = eye(dims.nbx, dims.nx);
lbx = [T_w_min; T_r_min; v_min; -1 * ones(2*numHiddenUnits,1)];
ubx = [T_w_max; T_r_max; v_max; ones(2*numHiddenUnits,1)];

% input bounds
Jbu = eye(dims.nbu);
lbu = [M_EM_acc_min; M_EM_brk_min; M_fric_brk_min];
ubu = [M_EM_acc_max; M_EM_brk_max; M_fric_brk_max];

lh = [0;0;P_EM_min;0]; %S04_v8
uh = [0;0;P_EM_max;50];

% soft nonlinear constraints h
Jsh = eye(dims.nh, dims.nsh);
Jsh(1, 1) = 1.0;
Jsh(2, 2) = 1.0;
Jsh(3, 3) = 1.0;
Jsh(4, 4) = 1.0;

%% acados ocp model

ocp_model = acados_ocp_model();

ocp_model.set('name', model_name);
% --- dims
ocp_model.set('T', dims.T);
ocp_model.set('dim_nx', dims.nx);
ocp_model.set('dim_nu', dims.nu);
ocp_model.set('dim_ny', dims.ny);
% ocp_model.set('dim_ny_e', dims.ny_e);
ocp_model.set('dim_nbx', dims.nbx);
% ocp_model.set('dim_nbx_e', dims.nbx_e);
ocp_model.set('dim_nbu', dims.nbu);
ocp_model.set('dim_nh', dims.nh);

ocp_model.set('dim_ns', dims.ns);

ocp_model.set('dim_nsh', dims.nsh);

ocp_model.set('dim_np', dims.np);

% --- symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('dyn_type', 'discrete');
ocp_model.set('dyn_expr_phi', model.expr_phi);
% ocp_model.set('sym_xdot', model.sym_xdot);
ocp_model.set('sym_p', model.sym_p);


% --- cost
ocp_model.set('cost_type', cost_type);
ocp_model.set('cost_type_e', cost_type);
ocp_model.set('cost_type_0', 'linear_ls'); % PS

if (strcmp(cost_type, 'linear_ls'))
	ocp_model.set('cost_Vu', Vu);
	ocp_model.set('cost_Vx', Vx);
else % nonlinear_ls
	ocp_model.set('cost_expr_y', model.expr_y);
	ocp_model.set('cost_expr_y_e', model.expr_y_e);
end
ocp_model.set('cost_W', W);
ocp_model.set('cost_W_0', W_0); % PS

% PS
ocp_model.set('cost_Vx_0', Vx_0);
ocp_model.set('cost_Vu_0', Vu_0);

ocp_model.set('cost_y_ref', yref);
ocp_model.set('cost_y_ref_0', yref_0);

% PS end

% % state bounds
ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_lbx', lbx);
ocp_model.set('constr_ubx', ubx);
% % input bounds
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', lbu);
ocp_model.set('constr_ubu', ubu);
% % nonlinear constraints
ocp_model.set('constr_expr_h', model.expr_h);
ocp_model.set('constr_lh', lh);
ocp_model.set('constr_uh', uh);

% ocp_model.set('constr_Jbx_e', Jbx_e);
% ocp_model.set('constr_lbx_e', lbx_e);
% ocp_model.set('constr_ubx_e', ubx_e);

ocp_model.set('constr_Jsh', Jsh);

% ocp_model.set('constr_x0', options.x0); % Initital state


ocp_model.model_struct

%% acados ocp opts

% ocp_opts = acados_ocp_opts();
% 
% % Orignal setiing solver options 
% ocp_opts.set('compile_interface', compile_interface);
% ocp_opts.set('codgen_model', codgen_model);
% 
% ocp_opts.set('param_scheme_N', N);
% 
% % if (exist('shooting_nodes', 'var'))
% % 	ocp_opts.set('param_scheme_shooting_nodes', shooting_nodes);
% % end
% 
% ocp_opts.set('nlp_solver', nlp_solver);
% 
% % if (exist('shooting_nodes', 'var'))
% % 	ocp_opts.set('param_scheme_shooting_nodes', shooting_nodes);
% % end
% 
% % if (strcmp(nlp_solver, 'sqp')) % not available for sqp_rti
% % ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
% % ocp_opts.set('nlp_solver_tol_stat', tol);
% % ocp_opts.set('nlp_solver_tol_eq', tol);
% % ocp_opts.set('nlp_solver_tol_ineq', tol);
% % ocp_opts.set('nlp_solver_tol_comp', tol);
% % end
% ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian);
% ocp_opts.set('regularize_method', regularize_method);
% ocp_opts.set('nlp_solver_ext_qp_res', nlp_solver_ext_qp_res);
% if (strcmp(nlp_solver, 'sqp'))
% 	ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
% end
% ocp_opts.set('qp_solver', qp_solver);
% ocp_opts.set('qp_solver_iter_max', qp_solver_max_iter);
% ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
% ocp_opts.set('qp_solver_cond_ric_alg', qp_solver_cond_ric_alg);
% if (contains(qp_solver, 'partial_condensing'))
% 	ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
% end
% if (strcmp(qp_solver, 'partial_condensing_hpipm'))
% 	ocp_opts.set('qp_solver_ric_alg', qp_solver_ric_alg);
% end
% ocp_opts.set('sim_method', sim_method);
% ocp_opts.set('print_level', qp_solver_print_level);
% % ocp_opts.opts_struct
% % Orignal Solver options end
%% acados ocp set opts (New ones)


ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('sim_method', sim_method);

ocp_opts.set('sim_method_num_stages', 2);
ocp_opts.set('sim_method_num_steps', 5);

ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_cond_N', N);
ocp_opts.set('print_level', 0);
ocp_opts.set('ext_fun_compile_flags', '');

% --- acados ocp, create ocp
ocp_opts.set('output_dir', fullfile('00_temp', ['build_acados_', date()]));
estimator = acados_ocp(ocp_model, ocp_opts);

disp('Model set up successfully');

%% Code Generation TODO

% if skip_code_generation == 0
%     codegen_mpc(go_on_scalexio, userpath, pathstr, ocp, model_name, options, description); % we do not need that for now
% end % skip_code_generation

