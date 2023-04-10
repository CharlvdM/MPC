% Setup script for  Assignment 5
% This script is called automatically at the start of the 
% Simulink MPC simulation, to initialize the plant and
% control parameters. This script can also be run 
% independantly to inspect these parameters and to see the 
% step response of the plant.

clear;
addpath('matlab2tikz\src'); % library that enables saving data for Latex pgf figures
savePlotData = true;

%% setup system variables
Ts = 1;     % sampling time
Nb = 6;     % blocking samples
Nc = 3;     % control moves
C = Nc*Nb;  % control horizon in terms of samples (not time)
Np = 100;    % prediction horizon
Q = [1 0; 0 1.4]; % output weighing matrix
R = [0.01 0; 0 0.04]; % input weighing matrix
% R = [0.1 0; 0 0.4]; % input weighing matrix
% n = 4;      % amount of states
p = 2;      % amount of inputs
q = 2;      % amount of outputs
tend = 250; % simulation end time
alpha = 0.3; % disturbance filter control

%% Create setpoint array. Note that time t = 0 is at index 1
YspArr = zeros(2,tend+1);
YspArr(:,150+1:tend+1) = [5;-5]*ones(1,tend-150+1);

% % to test
% Ysp = [5;-5]*ones(1,201);

%% Create prediction plant model
G11 = tf(12.8,[16.7 1],'IODelay', 1);
G12 = tf(-18.9,[21 1],'IODelay', 3);
G21 = tf(6.6,[10.9 1],'IODelay', 7);
G22 = tf(-19.4,[14.4 1],'IODelay', 3);
G = [G11 G12; G21 G22];
Gd = c2d(G,Ts); % convert to discrete model

% create a state space model which absorbs the delay
plantD = absorbDelay(ss(Gd));
n = size(plantD.A, 1);          % amount of states

%% Kalman filter 
% This process model model includes disturbance as a state
Abar = [plantD.A, plantD.B; zeros(p,n), eye(p)];
Bbar = [plantD.B, zeros(n,p); zeros(p,p), eye(p)];
Cbar = [plantD.C, zeros(q,p)];
Dbar = zeros(q, 2*p);

plantD_KF_ss = ss(Abar,Bbar,Cbar,Dbar,Ts);

% Determine Kalman gain
Q_KF = [1 0; 0 1]; 
R_KF = [50 0; 0 50];
[kalmf,L,P,M] = kalman(plantD_KF_ss,Q_KF,R_KF);

% State Space matrices for KF
I = eye(n+p);
A_KF = Abar * (I-M*Cbar);
B_KF = [Bbar, Abar*M];
C_KF = eye(16);
D_KF = zeros(16,6);
% C_KF = I - M*Cbar;
% D_KF = [zeros(n+p, 2*p), M];

%% Create process plant model
Gp11 = tf(1.2*12.8,[16.7 1],'IODelay', 1);
Gp12 = tf(-18.9,[1.2*21 1],'IODelay', 3);
Gp21 = tf(1.2*6.6,[10.9 1],'IODelay', 7);
Gp22 = tf(-19.4,[1.2*14.4 1],'IODelay', 3);
Gp = [Gp11 Gp12; Gp21 Gp22];

% %% Step Response
% figure(4); % generates Figure 4 in the report
% step(G)
% 
% if savePlotData == true
%     set(gcf,'Position',[200 200 600 400])
%     saveas(gcf,[pwd '\Figures\StepResponse.jpg']);
%     %matlab2tikz('Figures\StepResponse.tex');
% end
