% MPC test script

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

% random test setpoints
Ysp = [5;-5]; % random test setpoints

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

%% Setup the system
delayMod.A = A;
delayMod.B1 = B1;
delayMod.B2 = B2;
delayMod.C1 = C1;
delayMod.C2 = C2;
delayMod.D11 = D11;
delayMod.D12 = D12;
delayMod.D21 = D21;
delayMod.D22 = D22;
delayMod.tau = tau;
delayMod.inputDelay = plantD.inputDelay;
delayMod.n = n;
delayMod.p = p;
delayMod.q = q;
delayMod.Ysp = Ysp;

sysVar.Np = Np;
sysVar.Nb = Nb;
sysVar.Nc = Nc;
sysVar.Q = Q;
sysVar.R = R;
sysVar.Ts = Ts;
sysVar.C = C;

storedData.uPrevious = ones(p,Nc);
storedData.uStored = zeros(p,max(plantD.inputDelay)+1);
storedData.zk = zeros(1,tau+1);

xk = [0;0;0;0]; % initial states

%% Test MPC
tic
uOptimal = mpc(delayMod, sysVar, xk, storedData)
toc

[yk, uk, z1] = ...
    prediction(uOptimal, delayMod, sysVar, xk, storedData);

figure(5);
% plot(0:Np, yk, 0:Np, uk);
stairs(0:Np, yk');
% plot(0:Np, yk);
hold on
stairs(0:Np, uk');
hold off
xlim([0,60]);
title('MPC Optimized Input and Predicted Output');
xlabel('Samples from system discrete time k');
leg = legend('$y_1^*(k|k)$', '$y_2^*(k|k)$', ...
'$u_1^*(k|k)$', '$u_2^*(k|k)$', ...
'Location','east');
set(leg, 'Interpreter', 'latex');
set(gcf,'Position',[200 200 600 400])

if savePlotData == true
    matlab2tikz('Figures\TestMPC.tex');
end
