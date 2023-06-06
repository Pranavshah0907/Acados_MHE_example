function [estimator] = setup_estimator(model_mhe)

N = 20;
h = 0.05;
Fmax = 80;
T = N*h;

Q0_mhe = diag([0.1, 0.1, 0.1, 0.1, 1]);
Q_mhe = 10 * diag([0.2, 0.2, 2, 2, 0.1]);
R_mhe = 2 * diag([0.1, 0.1, 0.1, 0.1]);

%% Acados OCP model
ocp_mhe = acados_ocp_model();

nx_augmented = 5;
nu = 5; %nw
nparam = 1;
nx = 4;

ny = length(R_mhe) + length(Q_mhe); % h(x), w
ny_e = 0;
ny_0 = length(R_mhe) + length(Q_mhe) + length(Q0_mhe); % h(x), w and arrival cost

%% arguments
compile_interface = 'true'; %'auto';
codgen_model = 'true';
gnsf_detect_struct = 'true';

%% args

nlp_solver = 'sqp';
nlp_solver_max_iter = 200;
nlp_solver_exact_hessian = 'false';
regularize_method = 'project_reduc_hess';
qp_solver = 'partial_condensing_hpipm';
qp_solver_cond_N = 5;
qp_solver_cond_ric_alg = 0;
qp_solver_ric_alg = 0;
qp_solver_warm_start = 2;
qp_solver_max_iter = 100;

sim_method = 'erk';
% sim_method = irk;

cost_type = 'nonlinear_ls';
sim_method_num_stages = 4;
sim_method_num_steps = 3;

%% acados MHE ocp model

ocp_mhe.set('name', model_mhe.name);
ocp_mhe.set('T', T); % horizon end time length

ocp_mhe.set('sym_x', model_mhe.sym_x);
if isfield(model_mhe, 'sym_u')
	ocp_mhe.set('sym_u', model_mhe.sym_u);
end 

if isfield(model_mhe, 'sym_xdot')
	ocp_mhe.set('sym_xdot', model_mhe.sym_xdot);
end

% p_0 = 1;
ocp_mhe.set('sym_p', model_mhe.sym_p); %is this req? #done 

ocp_mhe.set('cost_type', 'nonlinear_ls');
ocp_mhe.set('cost_type_e', 'linear_ls');
ocp_mhe.set('cost_type_0', 'nonlinear_ls');

W_0 = blkdiag(Q_mhe, R_mhe, Q0_mhe);
ocp_mhe.set('cost_W_0', W_0);

% cost intermediate stages
W =  blkdiag(Q_mhe, R_mhe);
ocp_mhe.set('cost_W', W);

ocp_mhe.set('cost_expr_y_0', model_mhe.cost_expr_y_0);
ocp_mhe.set('cost_expr_y', model_mhe.cost_expr_y);


% dynamics
if (strcmp(sim_method, 'erk'))
	ocp_mhe.set('dyn_type', 'explicit');
	ocp_mhe.set('dyn_expr_f', model_mhe.expr_f_expl);
else % irk irk_gnsf
	ocp_mhe.set('dyn_type', 'implicit');
	ocp_mhe.set('dyn_expr_f', model_mhe.expr_f_impl);
end

% SET INITIAL PARAMETER VALUES MISSING HERE?

% set y_ref for all stages
yref = zeros(ny,1);
yref_e = zeros(ny_e,1);
yref_0 = zeros(ny_0,1);

ocp_mhe.set('cost_y_ref', yref);
ocp_mhe.set('cost_y_ref_e', yref_e);
ocp_mhe.set('cost_y_ref_0', yref_0);

disp('ocp_mhe');
disp(ocp_mhe.model_struct);


%% acados MHE ocp opts

ocp_mhe_opts = acados_ocp_opts();

ocp_mhe_opts.set('compile_interface', compile_interface);
ocp_mhe_opts.set('codgen_model', codgen_model);
ocp_mhe_opts.set('param_scheme_N', N);

ocp_mhe_opts.set('regularize_method', regularize_method);

ocp_mhe_opts.set('qp_solver', qp_solver);
ocp_mhe_opts.set('qp_solver_cond_ric_alg', qp_solver_cond_ric_alg);
ocp_mhe_opts.set('qp_solver_warm_start', qp_solver_warm_start);
ocp_mhe_opts.set('qp_solver_iter_max', qp_solver_max_iter);
if (~isempty(strfind(qp_solver, 'partial_condensing')))
	ocp_mhe_opts.set('qp_solver_cond_N', qp_solver_cond_N);
end
if (strcmp(qp_solver, 'partial_condensing_hpipm'))
	ocp_mhe_opts.set('qp_solver_ric_alg', qp_solver_ric_alg);
end

ocp_mhe_opts.set('nlp_solver', nlp_solver);
ocp_mhe_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian);
ocp_mhe_opts.set('sim_method', sim_method);


if (strcmp(nlp_solver, 'sqp'))
	ocp_mhe_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
end

disp('ocp_mhe_opts');
disp(ocp_mhe_opts.opts_struct);

estimator = acados_ocp(ocp_mhe, ocp_mhe_opts);

estimator.set('cost_W', W_0, 0);

end

