function [yk, uk, z1] = ...
    prediction(uControl, delayMod, sysVar, xk, storedData)

% Extract needed system variables
Np = sysVar.Np;
Nb = sysVar.Nb;
C = sysVar.C;

% Extract needed model parameters
inputDelay = delayMod.inputDelay;
A = delayMod.A;
B1 = delayMod.B1;
B2 = delayMod.B2;
C1 = delayMod.C1;
C2 = delayMod.C2;
D11 = delayMod.D11;
D12 = delayMod.D12;
D21 = delayMod.D21;
D22 = delayMod.D22;
tau = delayMod.tau;
p = delayMod.p;

% Stored data
uStored = storedData.uStored;
zk = storedData.zk;

% Implement blocking
uk = zeros(p,Np+1);
for i = 1:C
    index = floor((i-1)/Nb + 1);
    uk(:,i) = uControl(:,index);
end
uk(:,C+1:Np+1) = uControl(:,end)*ones(1,Np+1-C); % propagate last control move

% Create vector to store delayed inputs
ukDelayed = zeros(p,1);

% Create predicted output vector array
yk = zeros(2, Np+1);

%% Propagate state space model to implement prediction
for i = 1:Np+1
    % execute delay section of the state space model
    uStored(:,1) = uk(:,i); % if inputDelay is zero, then ukDelayed = uk(:,i)
    for j = 1:p
        ukDelayed(j) = uStored(j,inputDelay(j)+1);
    end
    zkminusTau = zk(tau+1);
    wk = zkminusTau;
    zk(1) = C2*xk+D21*ukDelayed+D22*wk;
    
    % store the value of z at thes start of simulation 
    % iteration to fill the z buffer for the next simulation 
    % iterations
    if i == 1
        z1 = zk(1);
    end
    
    % calculate states and output
    xkplus1 = A*xk+B1*ukDelayed+B2*wk;
    yk(:,i) = C1*xk+D11*ukDelayed+D12*wk;

    % setup parameters for next iteration
    xk = xkplus1;
    uStored = circshift(uStored,1,2);
    zk = circshift(zk,1);

end

end