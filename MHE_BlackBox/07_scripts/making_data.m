load("input_for_MHE.mat")
T_wr_orig = ts2timetable(T_wr_ideal_data);
T_wr_orig = T_wr_orig(1:10:end,:); % orignal data is at 0.01 timestep but the controls are at 0.1 time step

torque_cmd = ts2timetable(torque_cmd_data);
all_torque = torque_cmd.("mv (torque cmd)");
M_acc = all_torque(:,1);
M_brk = all_torque(:,2);
M_fric = all_torque(:,3);

torque_data(1,:) = M_acc(:,1); 
torque_data(2,:) = M_brk(:,1); 
torque_data(3,:) = M_fric(:,1); 

save("input_data_processed.mat")


% T_w = T_wr_orig.Data(:,1);

% T_w_noisy = awgn(T_w,10);
% 
% T_r = T_wr_orig.Data(:,2);
% T_r_noisy = awgn(T_r,10);
% 
% T_wr_noisy(1,:) = T_w_noisy(:,1);
% T_wr_noisy(2,:) = T_r_noisy(:,1);
% 
% 


% A_wnoise = A + 5*randn(size(A)) + 5;

% T_w_noisy2 = T_w + 0.1*randn(size(T_w)) - (0.02*T_w);
% 
% T_wr_noisy2 = T_wr_orig.Data + 0.1*randn(size(T_w)) - (0.01*T_wr_orig.Data);
% 
% plot(T_wr_noisy2)
% hold on
% plot(T_wr_orig.Data)