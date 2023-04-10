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
Np = 100 + C;    % prediction horizon
Q = [1 0; 0 1.4]; % output weighing matrix
R = [0.01 0; 0 0.04]; % input weighing matrix
n = 4;      % amount of states
p = 2;      % amount of inputs
q = 2;      % amount of outputs

% C and D to output states
CforStateOutput = eye(4);
DforStateOutput = zeros(n, p);

%% Create setpoint array. Note that time t = 0 is at index 1
Ysp = zeros(2,201);
Ysp(:,20+1:99+1) = [5;-5]*ones(1,80);
Ysp(:,100+1:200+1) = [-5;5]*ones(1,101);

% % to test
% Ysp = [5;-5]*ones(1,201);

%% Create plant model
G11 = tf(12.8,[16.7 1],'IODelay', 1);
G12 = tf(-18.9,[21 1],'IODelay', 3);
G21 = tf(6.6,[10.9 1],'IODelay', 7);
G22 = tf(-19.4,[14.4 1],'IODelay', 3);
G = [G11 G12; G21 G22];
Gd = c2d(G,Ts); % convert to discrete model

plantC = ss(G);
plantD = ss(Gd);    % discrete state-space

% decompose state-space system
[H,tau] = getDelayModel(plantD);
[A,B1,B2,C1,C2,D11,D12,D21,D22,E,tau]=getDelayModel(plantD);

%% Step Response
figure(4); % generates Figure 4 in the report
step(G)

if savePlotData == true
    set(gcf,'Position',[200 200 600 400])
    saveas(gcf,[pwd '\Figures\StepResponse.jpg']);
    %matlab2tikz('Figures\StepResponse.tex');
end
