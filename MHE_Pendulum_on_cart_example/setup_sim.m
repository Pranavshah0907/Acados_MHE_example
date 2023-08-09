function [sim] = setup_sim(model)

% general
N = 20;
h = 0.05;
Fmax = 80;

% NOTE: hard coded in export_pendulum_ode_model;
l_true = 0.8;

Q_ocp = diag([1e3, 1e3, 1e-2, 1e-2]);
R_ocp = 1e-2;

W = blkdiag(Q_ocp, R_ocp);

%% dims

T = N*h; % horizon length time
nx = model.nx;
nu = model.nu;
ny = model.nu+model.nx; % number of outputs in lagrange term
ny_e = model.nx; % number of outputs in mayer term

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
% sim_method = 'irk';

cost_type = 'nonlinear_ls';
sim_method_num_stages = 4;
sim_method_num_steps = 3;

model_name = 'pendulum_ocp';

%% constraints

nbu = 0;
lbu = -80*ones(nu, 1);
ubu =  80*ones(nu, 1);
x0 = [0; pi; 0; 0];
Jbu = 0;

% Jbu = zeros(nbu, nu); 
% for ii=1:nbu 
%     Jbu(ii,ii)=1.0; 
% end

%% Acados ocp model

ocp_model = acados_ocp_model();

ocp_model.set('name', model.name);
ocp_model.set('T', T); % horizon end time length

ocp_model.set('cost_expr_y',model.cost_expr_y);
ocp_model.set('cost_expr_y_e',model.cost_expr_y_e);

yref = zeros(ny, 1); % output reference in lagrange term
yref_e = zeros(ny_e, 1); % output reference in mayer term

% symbolics
ocp_model.set('sym_x', model.sym_x);
if isfield(model, 'sym_u')
	ocp_model.set('sym_u', model.sym_u);
end
if isfield(model, 'sym_xdot')
	ocp_model.set('sym_xdot', model.sym_xdot);
end

ocp_model.set('cost_type', cost_type);
ocp_model.set('cost_type_e', cost_type);

ocp_model.set('cost_W', model.W);
ocp_model.set('cost_W_e', model.W_e);

ocp_model.set('cost_y_ref', yref);
ocp_model.set('cost_y_ref_e', yref_e);

% dynamics
if (strcmp(sim_method, 'erk'))
	ocp_model.set('dyn_type', 'explicit');
	ocp_model.set('dyn_expr_f', model.expr_f_expl);
else % irk irk_gnsf
	ocp_model.set('dyn_type', 'implicit');
	ocp_model.set('dyn_expr_f', model.expr_f_impl);
end

ocp_model.set('constr_x0', x0);
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', lbu);
ocp_model.set('constr_ubu', ubu);

disp('ocp_model.model_struct')
disp(ocp_model.model_struct)


%% acados ocp opts

ocp_opts = acados_ocp_opts();

ocp_opts.set('compile_interface', compile_interface);
ocp_opts.set('codgen_model', codgen_model);
ocp_opts.set('param_scheme_N', N);

ocp_opts.set('regularize_method', regularize_method);

ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_cond_ric_alg', qp_solver_cond_ric_alg);
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

disp('ocp_opts');
disp(ocp_opts.opts_struct);

%% acados ocp

sim = acados_ocp(ocp_model, ocp_opts);
end

