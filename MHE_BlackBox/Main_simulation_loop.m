%% The Main Simulation loop

% THE CODE CAN BE STOPPED IN BETWEEN AND THE SUBSECTION OF THE PLOTS CAN BE
% RIN TO SEE THE RESULTS

load("input_data_processed.mat")

N_sim = 1000;

mean_m = 0; % Percentage error mean
st_d_m = 0.2; % Standard deviation of error

T_wr_noisy2 = T_wr_orig.Data + st_d_m*randn(size(T_wr_orig.Data)) - (mean_m*T_wr_orig.Data);
T_wr_noisy2 = T_wr_noisy2';

u_sim = torque_data;
y_sim = T_wr_noisy2;

x_est = zeros(dims.nx, N_sim+1);
u_est = zeros(dims.nu, N_sim);
% x_est(:,1) = options.x0;

yref_0 = zeros(dims.ny + dims.nu + dims.nx, 1);
yref = zeros(dims.ny + dims.nu, 1);

x0 = options.x0;
u0 = [80 0 0];

mheiter = 1;

for n = 1:N_sim-N

    % Setting the yref_0

    yref_0(1:dims.ny) = y_sim(:, n);
    yref_0(dims.ny+1:dims.ny+dims.nu) = u0;
    yref_0(dims.ny+dims.nu+1:end) = x0;

    estimator.set('cost_y_ref', yref_0, 0);
    
    % Get the measurements and update y_ref for the MHE horizon
    for i=1:N-1
        yref(1:dims.ny) = y_sim(:, n+i);
        yref(dims.ny+1:dims.ny+dims.nu) = u_sim(:,n+i);
        estimator.set('cost_y_ref', yref, i);
    end

    estimator.solve();
    
    x_est(:, n) = estimator.get('x', N);
    x0 = estimator.get('x', 1);

    u_est(:, n) = estimator.get('u', N-1);
    u0 = estimator.get('u', 1);

    disp(mheiter);
    mheiter = mheiter + 1;

end

%% Plot results

x_est_clipped = x_est(1,1:N_sim-options.P);
est_time = options.P+1:1:N_sim;
T_w_orig = (T_wr_orig.Data(1:N_sim,1))';

% Loading recorded outputs from LSMT
load('output_LSTM_orig.mat')
T_w_LSTM = T_w_LSTM(1:10:end);

figure;
plot(est_time,x_est_clipped);
hold on
plot(y_sim(1,1:N_sim))
plot(T_w_orig)
plot(T_w_LSTM(1:N_sim))

legend('est','meas','true','pred')
   
figure;
plot(u_sim(1,1:N_sim))
hold on
u_est_time = options.P+1:1:N_sim+options.P;
plot(u_est_time,u_est(1,:))

legend('sim','est')

% comments = 'W_is_10y_1u_short';
% save(['06_Results\mhe_',comments,datestr(now,'yyyymmdd_HHMM')])