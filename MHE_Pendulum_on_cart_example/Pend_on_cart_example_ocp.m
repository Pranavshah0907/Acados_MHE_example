clear all; clc;

%% test of native matlab interface
clear VARIABLES

% check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	error('env.sh has not been sourced! Before executing this example, run: source env.sh');
end

% general
N = 20;
h = 0.05;
Fmax = 80;

% NOTE: hard coded in export_pendulum_ode_model;
l_true = 0.8;

model = export_pendulum_ode_model;
sim = setup_sim(model);

nx = model.nx;
nu = model.nu;

%% acados ocp

sim
disp('sim.C_ocp');
disp(sim.C_ocp);
disp('sim.C_ocp_ext_fun');
disp(sim.C_ocp_ext_fun);

%% Simulation

% set trajectory initialization
% x_traj_init = zeros(nx, N+1);
% for ii=1:N x_traj_init(:,ii) = [0; pi; 0; 0]; end
x_traj_init = [linspace(0, 0, N+1); linspace(pi, 0, N+1); linspace(0, 0, N+1); linspace(0, 0, N+1)];

u_traj_init = zeros(nu, N);

% if not set, the trajectory is initialized with the previous solution
sim.set('init_x', x_traj_init);
sim.set('init_u', u_traj_init);

% change number of sqp iterations
%ocp.set('nlp_solver_max_iter', 20);

% solve
tic;

% solve ocp
sim.solve();

time_ext = toc;
% TODO: add getter for internal timing
fprintf(['time for ocp.solve (matlab tic-toc): ', num2str(time_ext), ' s\n'])

% get solution
u = sim.get('u');
x = sim.get('x');

%% PLot Results

figure;
subplot(4,1,1);
plot(0:N, x(1,:));
legend('p')
subplot(4,1,2);
plot(0:N, x(2,:));
legend('theta')
subplot(4,1,3);
plot(0:N, x(3,:));
legend('v')
subplot(4,1,4);
plot(0:N, x(4,:));
legend('omega')

%% Save results for MHE

% save('inputs_for_MHE_loop.mat')
