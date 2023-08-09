%% || Shree ||

clear; clc; close all;
clear mex;

[projectRootDir,~,~] = fileparts(mfilename('fullpath'));
idcs = strfind(projectRootDir,'\'); % find \ string
userpath = projectRootDir(1:idcs(end)-1); % get parent directory

pathstr = '\MHE_newapproach';
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

closed_loop = false;
open_loop = true;

addpath(genpath('..\MHE_newapproach'))
addpath(genpath('..\acados\external\casadi-matlab'))
load("input_data.mat")              % contains the control inputs
init_modvehicle_mpc_IPG;            % Init Vehicle parameters
init_MPC_Constants_slacks_LSTM;     % Init constants MPC and Options P,T, etc % _v2 with Constraints

model_name = 'LSTM_v21_MHE'; 
model = ocp_MHE_model_NMPC_v8_LSTM;


%% arguments for solver

compile_interface = 'auto';
codgen_model = 'true';

% integrator type
sim_method = 'discrete'; % erk, irk, irk_gnsf

% NLP Solver
nlp_solver = 'sqp';
nlp_solver_exact_hessian = 'false';
regularize_method = 'no_regularize'; %'project_reduc_hess'
nlp_solver_max_iter = 200; %options.sqp_steps;
nlp_solver_ext_qp_res = 0;

% investigate all the below from
qp_solver = 'partial_condensing_hpipm'; % ToDo: Investigate
qp_solver_cond_N = 5; 
qp_solver_cond_ric_alg = 0;               % factorize hessian in the condensing: (0) no, (1) yes
qp_solver_ric_alg = 0;                    % default 0
qp_solver_warm_start = 2;                 % (0) cold start, (1) warm start primal variables, (2) warm start and dual variables
qp_solver_max_iter = 100;               %options.maxIter;       
qp_solver_print_level = 1;
% param_scheme = 'multiple_shooting'; % 'single_shooting'
% sim_method = 'erk';
% sim_method = 'irk';
% Till

N = options.P; % currently 15
cost_type = 'linear_ls'; %'linear_ls'; 

%% Dims

dims.Ts = options.Ts; % sampling time 0.01
dims.T = N*dims.Ts; % horizon length time [s]
dims.ny = length(model.expr_y);
dims.nu = options.n_controls;
dims.nx = options.n_states;
dims.ny_e = 0;

%% Cost for linear

nout = 4; % T_w, T_r, w_Tw, w_Tr
nout_0 = 6; % T_w, T_r, w_Tw, w_Tr, t_w, t_r

% state-to-output matrix in lagrange term
Vx = zeros(nout, dims.nx);
Vx(1, 1) = 1.0; % T_w
Vx(2, 2) = 1.0; % T_r 

% input-to-output matrix in lagrange term
Vu = zeros(nout, dims.nx);
Vu(3,1) = 1.0; % w_Tw
Vu(4,2) = 1.0; % w_Tr
% Vu(dims.ny+1:dims.ny+dims.nu, 1:dims.nu) = eye(dims.nu);

Vx_0 = zeros(nout_0, dims.nx);
Vx_0(1,1) = 1.0;
Vx_0(2,2) = 1.0;
Vx_0(5,1) = 1.0;
Vx_0(6,2) = 1.0;

Vu_0 = zeros(nout_0, dims.nx); 
Vu_0(3,1) = 1.0;
Vu_0(4,2) = 1.0;

% % simualtion
% W = [10 0 0 0 0; % y1
%     0 10 0 0 0;  % y2
%     0 0 1 0 0;  % u1
%     0 0 0 1 0;  % u2
%     0 0 0 0 1]; % u3
% 
% % W(1, 1) =  Q(3,3); % v
% % W(2, 2) =  R(1,1); % u1
% % W(3, 3) =  R(2,2); % u2
% % W(4, 4) =  R(3,3); % u3
% 
% W_0 = eye(nout_0, nout_0);

%% Weigting Matrices

Q0_mhe = diag([0.1, 0.1]);
Q_mhe = 10 * diag([0.2, 0.2]);
R_mhe = 2 * diag([0.1, 0.1]);

%% Constraints --> Discuss which ones are required
% 
% % % % state bounds terminal mayer term
% % Jbx_e = eye(dims.nbx, dims.nx);
% % lbx_e = [T_w_min; T_r_min; v_min; -1 * ones(2*numHiddenUnits,1)];
% % ubx_e = [T_w_max; T_r_max; v_max; ones(2*numHiddenUnits,1)];
% 
% state bounds % -->
Jbx = eye(dims.nx, dims.nx);
lbx = [T_w_min; T_r_min; v_min; -1 * ones(2*numHiddenUnits,1)];
ubx = [T_w_max; T_r_max; v_max; ones(2*numHiddenUnits,1)];
% 
% input bounds Contraints on the noise
Jbu = eye(35);
lbu = [0; 0; zeros(33,1)];
ubu = [100; 100; 100*ones(33,1)];


% lh = [0; 0; P_EM_min; 0]; %S04_v8
% uh = [0; 0; P_EM_max; 50];
% 
% % soft nonlinear constraints h
% Jsh = eye(dims.nh, dims.nsh);
% Jsh(1, 1) = 1.0;
% Jsh(2, 2) = 1.0;
% Jsh(3, 3) = 1.0;
% Jsh(4, 4) = 1.0;

%% Setting up the estimator

ocp_model = acados_ocp_model();

ocp_model.set('name', model_name);
ocp_model.set('T', dims.T);

ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('sym_p', model.sym_p);

ocp_model.set('cost_type', cost_type);
ocp_model.set('cost_type_e', 'linear_ls');% cost_type);
ocp_model.set('cost_type_0', cost_type);

W_0 = blkdiag(Q_mhe, R_mhe, Q0_mhe);% all theses matrices have to be set
ocp_model.set('cost_W_0', W_0);

W =  blkdiag(Q_mhe, R_mhe);
ocp_model.set('cost_W', W);

if (strcmp(cost_type, 'linear_ls'))
	ocp_model.set('cost_Vu', Vu);
	ocp_model.set('cost_Vx', Vx);
    ocp_model.set('cost_Vx_0', Vx_0);
    ocp_model.set('cost_Vu_0', Vu_0);
else % nonlinear_ls
	ocp_model.set('cost_expr_y', model.expr_y);
    ocp_model.set('cost_expr_y_0', model.expr_y_0);
    % ocp_model.set('cost_W_e', W_e); % PS for non linear
end

% As sim_method is discrete
ocp_model.set('dyn_type', 'discrete');
ocp_model.set('dyn_expr_phi', model.expr_phi);

% % % state bounds
% ocp_model.set('constr_Jbx', Jbx);
% ocp_model.set('constr_lbx', lbx);
% ocp_model.set('constr_ubx', ubx);
% % % input bounds
% ocp_model.set('constr_Jbu', Jbu);
% ocp_model.set('constr_lbu', lbu);
% ocp_model.set('constr_ubu', ubu);

% set y_ref for all stages
yref = zeros(nout, 1);
yref_0 = zeros(nout_0, 1);
yref_e = zeros(dims.ny_e,1);

ocp_model.set('cost_y_ref', yref);
ocp_model.set('cost_y_ref_e', yref_e);
ocp_model.set('cost_y_ref_0', yref_0);

ocp_model.set('constr_x0', options.x0);

disp('ocp_mhe');
disp(ocp_model.model_struct);

%% acados ocp set opts (New ones)

ocp_opts = acados_ocp_opts();

ocp_opts.set('compile_interface', compile_interface);
ocp_opts.set('codgen_model', codgen_model);
ocp_opts.set('param_scheme_N', N);

ocp_opts.set('regularize_method', regularize_method);

ocp_opts.set('qp_solver', qp_solver);
% ocp_opts.set('qp_solver_cond_ric_alg', qp_solver_cond_ric_alg); default therefore unused
ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
ocp_opts.set('qp_solver_iter_max', qp_solver_max_iter);

if (~isempty(strfind(qp_solver, 'partial_condensing')))
	ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
end
if (strcmp(qp_solver, 'partial_condensing_hpipm'))
	ocp_opts.set('qp_solver_ric_alg', qp_solver_ric_alg);
end

ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian);
ocp_opts.set('sim_method', sim_method);

if (strcmp(nlp_solver, 'sqp'))
	ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
end

% ocp_opts.set('sim_method_num_stages', 2);
% ocp_opts.set('sim_method_num_steps', 5);

% ocp_opts.set('qp_solver_cond_N', N);
% ocp_opts.set('print_level', 0);
% ocp_opts.set('ext_fun_compile_flags', '');

% --- acados ocp, create ocp
ocp_opts.set('output_dir', fullfile('00_temp', ['build_acados_', date()]));

disp('ocp_opts');
disp(ocp_opts.opts_struct);

estimator = acados_ocp(ocp_model, ocp_opts);

estimator.set('cost_W', W_0, 0);

disp('Model set up successfully');

%% The simulation loop

% nx = options.n_states;
% ny = 0;
% nw = options.n_states;
% 
% dims.ny = length(model.expr_y);
% dims.nu = options.n_controls;
% dims.nx = options.n_states;
% dims.ny_e = 0;

% v_std = [1; 1];
% y_sim = zeros(2,length(x_sim)); % 2 is the number of measurements
% 
% rng(1);
% 
% for n = 1:length(x_sim)
%     y_sim(:, n) = x_sim(:, n) + diag(v_std)*randn(nx, 1);
% end

if open_loop
    load("T_wr_orig_v2.mat")
    load('parameter_input_v2.mat')
    
    mean_m = 0; % Percentage error mean
    st_d_m = 0.2; % Standard deviation of error
    
    T_wr_noisy = T_wr_orig_v2 + st_d_m*randn(size(T_wr_orig_v2)) - (mean_m*T_wr_orig_v2);

    % u_sim = torque_data;
    y_sim = T_wr_noisy';
    
    x_est = zeros(N+1,dims.nx);
    w_est = zeros(N,dims.nx);
    
    x0_bar = [60.0, 60.0]; 
    
    estimator.set('print_level',1);
    
    yref_0 = zeros(nout_0, 1); 
    yref_0(1:2) = y_sim(:,1);
    yref_0(5:end) = x0_bar;
    
    % initialize y, p
    estimator.set('cost_y_ref', yref_0, 0);
    estimator.set('p', parameter_remake(:,1) , 0);
    
    for j=1:N-1
        yref = zeros(nout,1);
    
        yref(1:2) = y_sim(:,j);
        estimator.set('cost_y_ref', yref, j);
    
        estimator.set('p', parameter_remake(:,j), j);
    end
    
    temp = ones(1,32);
    x0_init = [60, 60, 1, temp];
    
    % % intitalize x trajectory
    % x_traj_init = repmat(x0_init', 1, N+1);
    % estimator.set('init_x', x_traj_init);
    
    tic;
    
    % solve ocp
    estimator.solve();
    
    status = estimator.get('status');
    disp(status);
    estimator.print('stat');
    
    % the acados return values are:
    % 0 – success
    % 1 – failure
    % 2 – maximum number of iterations reached
    % 3 – minimum step size in QP solver reached
    % 4 – qp solver failed
    
    time_ext = toc;
    
    fprintf(['time for estimator.solve (matlab tic-toc): ', num2str(time_ext), ' s\n'])
    
    w_est = estimator.get('u');
    x_est = estimator.get('x');

end


%% create data

% load('vel_phi_data.mat')
% load('input_data_processed.mat')
% 
% parameter_remake = [M_acc(1:40)'; M_brk(1:40)'; M_fric(1:40)'; final_data(2,:); final_data(1,:)];
% 

%% closed loop

if closed_loop
    N_sim = 1000;
    
    load("T_wr_orig_v2.mat")
    load('parameter_input_v2.mat')
    
    mean_m = 0; % Percentage error mean
    st_d_m = 0.2; % Standard deviation of error
    
    % T_wr_noisy2 = T_wr_orig.Data + st_d_m*randn(size(T_wr_orig.Data)) - (mean_m*T_wr_orig.Data);
    % T_wr_noisy2 = T_wr_noisy2';
    
    T_wr_noisy = T_wr_orig_v2 + st_d_m*randn(size(T_wr_orig_v2)) - (mean_m*T_wr_orig_v2);

    % u_sim = torque_data;
    y_sim = T_wr_noisy';
    
    % x_est = zeros(N+1,dims.nx);
    x_est = zeros(dims.nx, N_sim-N);
    w_est = zeros(N,dims.nx);
    
    x0_bar = [60.0; 60.0];
    
    % temp = ones(1,32);
    % x0_init = [60, 60, 1, temp];
    
    % % intitalize x trajectory
    % x_traj_init = repmat(x0_init', 1, N+1);
    % estimator.set('init_x', x_traj_init);
    
    estimator.set('print_level',1);
    
    tic;
    
    for i=1:N_sim-N
        yref_0 = zeros(nout_0, 1); 
        yref_0(1:2) = y_sim(:,1);
        yref_0(5:end) = x0_bar(1:2,1);
        
        % initialize y, p
        estimator.set('cost_y_ref', yref_0, 0);
        estimator.set('p', parameter_input_v2(:,1) , 0);
    
        for j=1:N-1
            yref = zeros(nout,1);
        
            yref(1:2) = y_sim(:,j+i);
            estimator.set('cost_y_ref', yref, j);
        
            estimator.set('p', parameter_input_v2(:,j+i), j);
        end
        
        % solve ocp
        estimator.solve();
        
        status = estimator.get('status');
        disp(status);
        estimator.print('stat');
        
        x_est(:, i) = estimator.get('x', N);
    
        % w_est(:, i) = estimator.get('u');
        x0_bar = estimator.get('x',1);
    
    end
    
    time_ext = toc;
    
    fprintf(['time for estimator.solve (matlab tic-toc): ', num2str(time_ext), ' s\n'])
    
    % the acados return values are:
    % 0 – success
    % 1 – failure
    % 2 – maximum number of iterations reached
    % 3 – minimum step size in QP solver reached
    % 4 – qp solver failed

end
