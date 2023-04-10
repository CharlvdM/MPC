function mpc_s_L2(block)
% s functoin implementation of Model Predictive Control. In
% each iteration, the block receives the current measurements
% of the output. The block calculates the optimal control 
% move, which it outputs.
% Dialog input parameters:
% - Gd      discrete plant model
% - Ysp     output setpoints over time
% - Np      prediction horizon
% - Nb      blocking samples
% - Nc      control moves
% - Q       output weighing matrix
% - R       input weighing matrix
% - Ts      sampling time

%% Main body: Call setup function. No other calls should be 
%% added to the main body.
setup(block);

%endfunction

%% Set up the basic characteristics of the S-function block
function setup(block)

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 2;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions  = 4;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% Override output port properties
block.OutputPort(1).Dimensions  = 2;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

block.OutputPort(2).Dimensions  = 2;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 8;

% Register sample times
Ts = block.DialogPrm(8).Data;
block.SampleTimes = [Ts 0];  % sample time of 1 s

% Specify the block simStateCompliance.
%    'DefaultSimState', < Same sim state as a built-in block
block.SimStateCompliance = 'DefaultSimState';

% Register all relevant methods
block.RegBlockMethod(...
    'PostPropagationSetup', @DoPostPropSetup);
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);
block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Terminate', @Terminate);
block.RegBlockMethod(...
    'SetInputPortSamplingMode', @SetInputPortSamplingMode);

%end setup

%% Setup work areas and state variables
function DoPostPropSetup(block)
block.NumDworks = 15;

%% Iteration counter vector
  block.Dwork(1).Name            = 'SimIteration';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
%% Plant model storage
Gd = block.DialogPrm(1).Data;
plantD = ss(Gd);    % discrete plant
[A,B1,B2,C1,C2,D11,D12,D21,D22,E,tau]=getDelayModel(plantD);

  Asize = size(A);
  block.Dwork(2).Name            = 'A';
  block.Dwork(2).Dimensions      = Asize(1) * Asize(2);
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;

  B1size = size(B1);
  block.Dwork(3).Name            = 'B1';
  block.Dwork(3).Dimensions      = B1size(1) * B1size(2);
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;

  B2size = size(B2);
  block.Dwork(4).Name            = 'B2';
  block.Dwork(4).Dimensions      = B2size(1) * B2size(2);
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;

  C1size = size(C1);
  block.Dwork(5).Name            = 'C1';
  block.Dwork(5).Dimensions      = C1size(1) * C1size(2);
  block.Dwork(5).DatatypeID      = 0;      % double
  block.Dwork(5).Complexity      = 'Real'; % real
  block.Dwork(5).UsedAsDiscState = true;

  C2size = size(C2);
  block.Dwork(6).Name            = 'C2';
  block.Dwork(6).Dimensions      = C2size(1) * C2size(2);
  block.Dwork(6).DatatypeID      = 0;      % double
  block.Dwork(6).Complexity      = 'Real'; % real
  block.Dwork(6).UsedAsDiscState = true;

  D11size = size(D11);
  block.Dwork(7).Name            = 'D11';
  block.Dwork(7).Dimensions      = D11size(1) * D11size(2);
  block.Dwork(7).DatatypeID      = 0;      % double
  block.Dwork(7).Complexity      = 'Real'; % real
  block.Dwork(7).UsedAsDiscState = true;

  D12size = size(D12);
  block.Dwork(8).Name            = 'D12';
  block.Dwork(8).Dimensions      = D12size(1) * D12size(2);
  block.Dwork(8).DatatypeID      = 0;      % double
  block.Dwork(8).Complexity      = 'Real'; % real
  block.Dwork(8).UsedAsDiscState = true;

  D21size = size(D21);
  block.Dwork(9).Name            = 'D21';
  block.Dwork(9).Dimensions      = D21size(1) * D21size(2);
  block.Dwork(9).DatatypeID      = 0;      % double
  block.Dwork(9).Complexity      = 'Real'; % real
  block.Dwork(9).UsedAsDiscState = true;

  D22size = size(D22);
  block.Dwork(10).Name            = 'D22';
  block.Dwork(10).Dimensions      = D22size(1) * D22size(2);
  block.Dwork(10).DatatypeID      = 0;      % double
  block.Dwork(10).Complexity      = 'Real'; % real
  block.Dwork(10).UsedAsDiscState = true;

  block.Dwork(11).Name            = 'tau';
  block.Dwork(11).Dimensions      = 1;
  block.Dwork(11).DatatypeID      = 0;      % double
  block.Dwork(11).Complexity      = 'Real'; % real
  block.Dwork(11).UsedAsDiscState = true;

inputDelay = plantD.inputDelay;
 
  inputDelaysize = size(inputDelay);
  block.Dwork(12).Name            = 'inputDelay';
  block.Dwork(12).Dimensions      = ...
      inputDelaysize(1) * inputDelaysize(2);
  block.Dwork(12).DatatypeID      = 0;      % double
  block.Dwork(12).Complexity      = 'Real'; % real
  block.Dwork(12).UsedAsDiscState = true;

%% Dynamic storage vectors
  block.Dwork(13).Name            = 'uPrevious';
  p = size(B1,2); % amount of inputs
  Dimension13 = p*block.DialogPrm(5).Data; % number of inputs
        % timesprediction samples
  block.Dwork(13).Dimensions      = Dimension13;
  block.Dwork(13).DatatypeID      = 0;      % double
  block.Dwork(13).Complexity      = 'Real'; % real
  block.Dwork(13).UsedAsDiscState = true;

  block.Dwork(14).Name            = 'uStored';
  Dimension14 = p*(max(inputDelay)+1); 
  block.Dwork(14).Dimensions      = Dimension14;
  block.Dwork(14).DatatypeID      = 0;      % double
  block.Dwork(14).Complexity      = 'Real'; % real
  block.Dwork(14).UsedAsDiscState = true;
  
  block.Dwork(15).Name            = 'zk';
  block.Dwork(15).Dimensions      = tau+1;
  block.Dwork(15).DatatypeID      = 0;      % double
  block.Dwork(15).Complexity      = 'Real'; % real
  block.Dwork(15).UsedAsDiscState = true;
 
% end DoPostPropSetup
  
%% Called at start of model execution to initialize states
function Start(block)

block.Dwork(1).Data = 1; % start simIter at 1

% determine the discrete state space plant parameters
Gd = block.DialogPrm(1).Data;
plantD = ss(Gd);    % discrete plant
[A,B1,B2,C1,C2,D11,D12,D21,D22,E,tau]=getDelayModel(plantD);

% store flattened matrixes and vectors (plant parameters)
block.Dwork(2).Data = reshape(A,[],1);
block.Dwork(3).Data = reshape(B1,[],1);
block.Dwork(4).Data = reshape(B2,[],1);
block.Dwork(5).Data = reshape(C1,[],1);
block.Dwork(6).Data = reshape(C2,[],1);
block.Dwork(7).Data = reshape(D11,[],1);
block.Dwork(8).Data = reshape(D12,[],1);
block.Dwork(9).Data = reshape(D21,[],1);
block.Dwork(10).Data = reshape(D22,[],1);
block.Dwork(11).Data = tau;
inputDelay = plantD.inputDelay;
block.Dwork(12).Data = reshape(inputDelay,[],1);

% initialize dynamic storage vectors
p = size(B1,2); % amount of inputs
Nc = block.DialogPrm(5).Data;
uGuess = ones(p,Nc);
uGuess = reshape(uGuess,[],1); % flatten matrix
block.Dwork(13).Data = uGuess; % first guess for control

uStored = zeros(p,max(inputDelay)+1);
block.Dwork(14).Data = reshape(uStored,[],1);

zk = zeros(1,tau+1);
block.Dwork(15).Data = zk;

%end Start

%% Called to generate block outputs in simulation step
function Outputs(block)

tic

%% Reshape stored parameters
Aflat = block.Dwork(2).Data;
n = sqrt(length(Aflat));  % amount of states
A = reshape(Aflat,n,n);

B1flat = block.Dwork(3).Data;
B1 = reshape(B1flat,n,[]);
p = size(B1,2); % amount of inputs

B2flat = block.Dwork(4).Data;
B2 = reshape(B2flat,n,1);

C1flat = block.Dwork(5).Data;
C1 = reshape(C1flat,[],n);
q = size(C1,1); % amount of outputs

C2flat = block.Dwork(6).Data;
C2 = reshape(C2flat,1,n);

D11flat = block.Dwork(7).Data;
D11 = reshape(D11flat,q,p);

D12flat = block.Dwork(8).Data;
D12 = reshape(D12flat,q,1);

D21flat = block.Dwork(9).Data;
D21 = reshape(D21flat,1,p);

D22 = block.Dwork(10).Data;

tau = block.Dwork(11).Data;

inputDelayFlat = block.Dwork(12).Data;
inputDelay = reshape(inputDelayFlat,p,1);

%% Create data structures to pass to functions
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
delayMod.inputDelay = inputDelay;
delayMod.n = n;
delayMod.p = p;
delayMod.q = q;

% Setpoint
Ysp = block.DialogPrm(2).Data;
simIter = block.Dwork(1).Data;
delayMod.Ysp = Ysp(:,simIter); % update the setpoint based on simulation time

sysVar.Np = block.DialogPrm(3).Data;
sysVar.Nb = block.DialogPrm(4).Data;
sysVar.Nc = block.DialogPrm(5).Data;
sysVar.Q = block.DialogPrm(6).Data;
sysVar.R = block.DialogPrm(7).Data;
sysVar.Ts = block.DialogPrm(8).Data;
sysVar.C = sysVar.Nb*sysVar.Nc;

%% Read current state
xk = block.InputPort(1).Data;

%% Reshape stored data and create a structure
uPrevious = block.Dwork(13).Data; % take the last control 
        % implemented as the first guess of the new step
uPrevious = reshape(uPrevious,p,[]);
uStoredFlat = block.Dwork(14).Data;
uStored = reshape(uStoredFlat,p,max(inputDelay)+1);
uStored = circshift(uStored,1,2);
zk = block.Dwork(15).Data;
zk = circshift(zk,1);

storedData.uPrevious = uPrevious;
storedData.uStored = uStored;
storedData.zk = zk;

%% Execute MPC algorithm to find optimal control
uOptimal = mpc(delayMod, sysVar, xk, storedData)

%% Store relevant data
block.Dwork(13).Data = reshape(uOptimal, [], 1);
uStored(:,1) = uOptimal(:,1);
block.Dwork(14).Data = reshape(uStored, [], 1);

% execute prediction once more to extract z1 and for plots
% for debugging
[yk, uk, z1] = ...
    prediction(uOptimal, delayMod, sysVar, xk, storedData);
zk(1) = z1;
block.Dwork(15).Data = zk;

toc

%% Plots for debugging
simTime1 = 20;
simTime2 = 80;
simTime3 = 100;
if simIter == simTime1+1 || simIter == simTime2+1 || ...
        simIter == simTime3+1
    if simIter == simTime1+1 % at time 20 seconds
        figure(1);
    elseif simIter == simTime2+1
        figure(2);
    elseif simIter == simTime3+1
        figure(8); % corresponds with figure number in report
    end
    stairs(0:sysVar.Np, yk');
    hold on
    stairs(0:sysVar.Np, uk');
    hold off
    title(strcat('MPC Optimized Input and Predicted', ...
        ' Output for Simulink Time at', {' '}, ...
        num2str(simIter-1)));
    xlabel(strcat('Samples from Simulink discrete time', ...
        {' '}, num2str(simIter-1)));
    leg = legend('$y_1^*(k|k)$', '$y_2^*(k|k)$', ...
    '$u_1^*(k|k)$', '$u_2^*(k|k)$', ...
    'Location','east');
    set(leg, 'Interpreter', 'latex');
    set(gcf,'Position',[200 200 600 400])
    if simIter == simTime3+1
        addpath('matlab2tikz\src'); % library that enables saving data for Latex pgf figures
        matlab2tikz('Figures\MPCduringSim.tex');
    end
end

%% Output optimal control step
block.OutputPort(1).Data = uOptimal(:,1);

%% Output calculated plant output for debugging
block.OutputPort(2).Data = yk(:,1); % output y(k|k)

%end Outputs

%% Called to update discrete states during simulation step
function Update(block)

block.Dwork(1).Data = block.Dwork(1).Data + 1;

%end Update

%% Set the sampling of the ports
function SetInputPortSamplingMode(block, idx, fd)

    block.InputPort(idx).SamplingMode = fd;
    for i = 1:block.NumOutputPorts
        block.OutputPort(i).SamplingMode = fd;
    end

%end SetInputPortSamplingMode
    
%% Called at the end of simulation for cleanup
function Terminate(block)

%end Terminate

