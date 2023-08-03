%% Dims to set up the cost

dims.Ts = options.Ts; % sampling time 0.01
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


%% MOdel setup extras

% % --- dims
% 
% ocp_model.set('dim_nx', dims.nx);
% ocp_model.set('dim_nu', dims.nu);
% ocp_model.set('dim_ny', dims.ny);
% % ocp_model.set('dim_ny_e', dims.ny_e);
% ocp_model.set('dim_nbx', dims.nbx);
% % ocp_model.set('dim_nbx_e', dims.nbx_e);
% ocp_model.set('dim_nbu', dims.nbu);
% ocp_model.set('dim_nh', dims.nh);
% 
% ocp_model.set('dim_ns', dims.ns);
% 
% ocp_model.set('dim_nsh', dims.nsh);
% 
% ocp_model.set('dim_np', dims.np);
% 
% % --- symbolics
% 
% 
% % ocp_model.set('sym_xdot', model.sym_xdot);
% 
% 
% 
% % --- cost
% 
% 
% if (strcmp(cost_type, 'linear_ls'))
% 	ocp_model.set('cost_Vu', Vu);
% 	ocp_model.set('cost_Vx', Vx);
% else % nonlinear_ls
% 	ocp_model.set('cost_expr_y', model.expr_y);
% 	ocp_model.set('cost_expr_y_e', model.expr_y_e);
% end
% ocp_model.set('cost_W', W);
% ocp_model.set('cost_W_0', W_0); % PS
% 
% % PS
% ocp_model.set('cost_Vx_0', Vx_0);
% ocp_model.set('cost_Vu_0', Vu_0);
% 
% ocp_model.set('cost_y_ref', yref);
% ocp_model.set('cost_y_ref_0', yref_0);
% 
% % PS end
% 
% % % state bounds
% ocp_model.set('constr_Jbx', Jbx);
% ocp_model.set('constr_lbx', lbx);
% ocp_model.set('constr_ubx', ubx);
% % % input bounds
% ocp_model.set('constr_Jbu', Jbu);
% ocp_model.set('constr_lbu', lbu);
% ocp_model.set('constr_ubu', ubu);
% % % nonlinear constraints
% ocp_model.set('constr_expr_h', model.expr_h);
% ocp_model.set('constr_lh', lh);
% ocp_model.set('constr_uh', uh);
% 
% % ocp_model.set('constr_Jbx_e', Jbx_e);
% % ocp_model.set('constr_lbx_e', lbx_e);
% % ocp_model.set('constr_ubx_e', ubx_e);
% 
% ocp_model.set('constr_Jsh', Jsh);
% 
% % ocp_model.set('constr_x0', options.x0); % Initital state
% 
% 
% ocp_model.model_struct