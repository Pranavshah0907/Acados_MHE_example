% clear all; clc;

%% MHE model and estimator

load('inputs_for_MHE_loop.mat') % Using pre recorded simualtion results

N = 20;
h = 0.05;
Fmax = 80;
T = N*h;

model_mhe = export_mhe_ode_model_with_noisy_param;
estimator = setup_estimator(model_mhe);

nx_augmented = 5;
nw = 5;
ny = 4;
nx = 4;

%% Estimation

u_sim = u;
x_sim = x;

%v_std = [0; 0; 0; 0];
v_std = [0.2; 0.5; 1; 1];

y_sim = zeros(4,length(x));

% fix randomness for reproducibility
rng(1);

for n = 1:length(x)
    y_sim(:, n) = x_sim(:, n) + diag(v_std)*randn(nx, 1);
end

x_est = zeros(N+1,nx);
w_est = zeros(N,nx_augmented);
l_est = zeros(N+1, 1);

x0_bar = [0.0, 0.0, 0.0, 0.0, 0.2];
% x0_bar = [0.0, pi, 0.0, 0.0, 1];

estimator.set('print_level',4);

yref_0 = zeros(nx + 2*nx_augmented, 1); 
yref_0(1:nx) = y_sim(:,1);
yref_0(nx+nx_augmented+1:end) = x0_bar;


% initialize y, p

estimator.set('cost_y_ref', yref_0, 0);
estimator.set('p', u_sim(1) , 0);

yref = zeros(2*nx,1);
for j=1:N-1

    yref = zeros(nx + nx_augmented,1);

    yref(1:nx) = y_sim(:,j);
    estimator.set('cost_y_ref', yref, j);
    estimator.set('p', u_sim(j), j);

end

% intitalize x trajectory
x_traj_init = repmat(x0_bar', 1, N+1);
estimator.set('init_x', x_traj_init);

tic;

% solve ocp
estimator.solve();

status = estimator.get('status');
estimator.print('stat');

time_ext = toc;

fprintf(['time for estimator.solve (matlab tic-toc): ', num2str(time_ext), ' s\n'])

w_est = estimator.get('u');
x_augmented = estimator.get('x');
% x_est = TODO get 1st 4 of x_augmented

%% Plot Results

figure;
subplot(4,1,1);
scatter(0:N, y_sim(1,:));
hold on
plot(0:N, x_sim(1,:));
plot(0:N, x_augmented(1,:));
title('p')
legend('Measurements','True','Estimates')

subplot(4,1,2);
scatter(0:N, y_sim(2,:));
hold on
plot(0:N, x_sim(2,:));
plot(0:N, x_augmented(2,:));
title('theta')
legend('Measurements','True','Estimates')

subplot(4,1,3);
scatter(0:N, y_sim(3,:));
hold on
plot(0:N, x_sim(3,:));
plot(0:N, x_augmented(3,:));
title('v')
legend('Measurements','True','Estimates')

subplot(4,1,4);
scatter(0:N, y_sim(4,:));
hold on
plot(0:N, x_sim(4,:));
plot(0:N, x_augmented(4,:));
title('omega')
legend('Measurements','True','Estimates')
