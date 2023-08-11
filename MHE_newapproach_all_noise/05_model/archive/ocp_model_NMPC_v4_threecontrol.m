%% BOF
function model = ocp_model_NMPC_v4_threecontrol()
import casadi.*
init_modvehicle_mpc_IPG_ac;    %Init Vehicle parameters
init_MPC_Constants_slacks_ac;     % Init constants MPC and Options P,T, etc
init_LPTN_mpc_ac;          %Init LPTN thermal model in vehicle model
% init_Param_Thermisches_Modell_EM; %Init thermal model M.Schmitz

%% define the symbolic variables of the plant
ocp_S02_DefACADOSVarSpace_ac_v4_threecontrol;

%% load plant parameters
% ocp_S03_SetupSysParameters_ac; % commentend out bc empty

%% define ode rhs in explicit form (22 equations)
% ocp_S04_SetupNonlinearStateSpaceDynamics_ac;
ocp_S04_SetupNonlinearStateSpaceDynamics_ac_v5_threecontrol;

%% generate casadi C functions
nx = options.n_states;
nu = options.n_controls;
np = options.n_parameter;

%% populate structure
model.nx = nx;
model.nu = nu;
model.np = np;
model.sym_x = x;
model.sym_xdot = dx;
model.sym_u = u;
model.sym_p = p;
model.expr_f_expl = f_expl;
model.expr_f_impl = f_impl;
model.expr_h = h;
model.expr_h_e = h_e;
model.expr_g = g;
model.expr_g_e = g_e;
model.expr_y = y;
model.expr_y_e = y_e;

