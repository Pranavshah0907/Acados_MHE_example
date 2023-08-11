%% Vehicle Model Initialization file
% File for initalisation of vehicle model

%% Parameter E-machine
% Load Parameter EM
load('param_maxtorque.mat');
load(fullfile([cd,'\02_parameter\data_efficiency_TPS_Gl1_60C_new_AW'])); %new, V15, 1403, GL1, Thin plate spline

%% Parameter Vehicle
r_dyn = 0.293/0.9624;% (m) tire diameter
grav = 9.81; %m/s^2
dens = 1.2041; %20C %1.1849;%(kg/m^3) @ 25 C density
Cd = 0.355; %0.27 %coefficent of drag
Af = 2.26; %m^2 %statt 2.19; %m^2
fr = 0.011; %coefficent of friction
FDR = 9.3; % 9.3 %final drive ratio
% FDR = 9.3/1.09528; % Differenz zw. IPG und Modell hier enthalten 
% eta_diff = 0.95; %differential efficiency not used
Mv = 1160; %kg 
Pi=pi;
torque_brake = 515;      %Nm brake torque % F_fzg_min = -21.46 kN mit a_min = -18,5 m/s² (aus IPG) und Mv (F_rad = MV * a), M_EM = F_Fzg * r_dyn / FDR
% M_EM_welle_brk_min = -675 N. abzgl. M_EM_brk_min = -160.63 Nm. M_fric_brk = -515 Nm
rps_rpm = 30/pi;     % rad/sec to rpm
rpm_rps = pi/30;
mps_kmh = 3.6;
kmh_mps = 1/3.6;

%% Efficiency map
temp_vec = [0, 25, 125, 175];
f_eff = [1, 1, 0.9, 0.85];
%%
% Driver model paramters
v_max = 130; % 130 km/h
v_max_c = v_max; %making v dimensionless

% driver controller parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% keep unchanged (Note: Jinming Liu)
Kf_c = 1/10;
Kp_c = 30;
Ti_c = 60;
Tt_c = 65;
% v_max_c = 100;