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
block.NumInputPorts  = 2;
block.NumOutputPorts = 2;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions  = 14;
% block.InputPort(1).Dimensions  = 16;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

block.InputPort(2).Dimensions  = 2;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;

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
block.NumDworks = 6;

%% Iteration counter vector
  block.Dwork(1).Name            = 'SimIteration';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
%% Plant model storage
Gd = block.DialogPrm(1).Data;
plantD = absorbDelay(ss(Gd));   % discrete state-space, delays absorbed
A = plantD.A;
B = plantD.B;
C = plantD.C;
D = plantD.D;

  block.Dwork(2).Name            = 'A';
  block.Dwork(2).Dimensions      = size(A,1) * size(A,2);
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;

  block.Dwork(3).Name            = 'B';
  block.Dwork(3).Dimensions      = size(B,1) * size(B,2);
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;

  block.Dwork(4).Name            = 'C';
  block.Dwork(4).Dimensions      = size(C,1) * size(C,2);
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;

  block.Dwork(5).Name            = 'D';
  block.Dwork(5).Dimensions      = size(D,1) * size(D,2);
  block.Dwork(5).DatatypeID      = 0;      % double
  block.Dwork(5).Complexity      = 'Real'; % real
  block.Dwork(5).UsedAsDiscState = true;

%% Dynamic storage
  block.Dwork(6).Name            = 'uPrevious';
  p = size(B,2); % amount of inputs
  Dimension6 = p*block.DialogPrm(5).Data; % number of inputs
        % timesprediction samples
  block.Dwork(6).Dimensions      = Dimension6;
  block.Dwork(6).DatatypeID      = 0;      % double
  block.Dwork(6).Complexity      = 'Real'; % real
  block.Dwork(6).UsedAsDiscState = true;
 
% end DoPostPropSetup
  
%% Called at start of model execution to initialize states
function Start(block)

block.Dwork(1).Data = 1; % start simIter at 1

% determine the discrete state space plant parameters
Gd = block.DialogPrm(1).Data;
plantD = absorbDelay(ss(Gd));   % discrete state-space, delays absorbed
A = plantD.A;
B = plantD.B;
C = plantD.C;
D = plantD.D;

% store flattened matrixes and vectors (plant parameters)
block.Dwork(2).Data = reshape(A,[],1);
block.Dwork(3).Data = reshape(B,[],1);
block.Dwork(4).Data = reshape(C,[],1);
block.Dwork(5).Data = reshape(D,[],1);

% initialize dynamic storage vector
p = size(B,2); % amount of inputs
Nc = block.DialogPrm(5).Data;
uGuess = ones(p,Nc);
uGuess = reshape(uGuess,[],1); % flatten matrix
block.Dwork(6).Data = uGuess; % first guess for control

%end Start

%% Called to generate block outputs in simulation step
function Outputs(block)

tic

%% Reshape stored parameters
Aflat = block.Dwork(2).Data;
n = sqrt(length(Aflat));  % amount of states
A = reshape(Aflat,n,n);

Bflat = block.Dwork(3).Data;
B = reshape(Bflat,n,[]);
p = size(B,2); % amount of inputs

Cflat = block.Dwork(4).Data;
C = reshape(Cflat,[],n);
q = size(C,1); % amount of outputs

Dflat = block.Dwork(5).Data;
D = reshape(Dflat,q,p);

%% Create data structures to pass to functions
plantD.A = A;
plantD.B = B;
plantD.C = C;
plantD.D = D;

% Setpoint
YspArr = block.DialogPrm(2).Data;
simIter = block.Dwork(1).Data;
Ysp = YspArr(:,simIter); % update the setpoint based on simulation time

sysVar.Np = block.DialogPrm(3).Data;
sysVar.Nb = block.DialogPrm(4).Data;
sysVar.Nc = block.DialogPrm(5).Data;
sysVar.Q = block.DialogPrm(6).Data;
sysVar.R = block.DialogPrm(7).Data;
sysVar.Ts = block.DialogPrm(8).Data;
sysVar.C = sysVar.Nb*sysVar.Nc;
sysVar.n = n;
sysVar.p = p;
sysVar.q = q;

%% Read current state
% zk = block.InputPort(1).Data;
% xk = zk(1:n);       % state estimate
% dk = zk(n+1:n+p); % disturbance estimate
xk = block.InputPort(1).Data;
dk = block.InputPort(2).Data;

%% Reshape previous control
uPrevious = block.Dwork(6).Data; % take the last control 
        % implemented as the first guess of the new step
uPrevious = reshape(uPrevious,p,[]);

%% Execute MPC algorithm to find optimal control
uOptimal = mpc(plantD, Ysp, sysVar, xk, dk, uPrevious)

%% Store control data
block.Dwork(6).Data = reshape(uOptimal, [], 1);

%% Do prediction once more for plotting and debugging
[yk, uk] = prediction(uOptimal, plantD, sysVar, xk, dk);

toc

%% Plots for debugging
simTime1 = 20;
simTime2 = 80;
simTime3 = 150;
if simIter == simTime1+1 || simIter == simTime2+1 || ...
        simIter == simTime3+1
    
%     [yk, uk] = prediction(uOptimal, plantD, sysVar, xk);
    
    if simIter == simTime1+1 % at time 20 seconds
        figure(1);
    elseif simIter == simTime2+1
        figure(2);
    elseif simIter == simTime3+1
        figure(3); % corresponds with figure number in report
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

