function [yk, uk] = ...
    prediction(uControl, plantD, sysVar, xk, dk)
% Extract needed system variables
Np = sysVar.Np;
Nb = sysVar.Nb;
CH = sysVar.C; % control horizon
p = sysVar.p;

% Extract model parameters
A = plantD.A;
B = plantD.B;
C = plantD.C;
D = plantD.D;

% Implement blocking
uk = zeros(p,Np+1);
for i = 1:CH
    index = floor((i-1)/Nb + 1);
    uk(:,i) = uControl(:,index);
end
uk(:,CH+1:Np+1) = ...
    uControl(:,end)*ones(1,Np+1-CH); % propagate last control

% Create predicted output vector array
yk = zeros(2, Np+1);

%% Propagate state space model to implement prediction
for i = 1:Np+1
    % calculate states and output
    xkplus1 = A*xk+B*(uk(:,i)+dk);
    yk(:,i) = C*xk+D*(uk(:,i)+dk);
    
    % setup states for the next iteration
    xk = xkplus1;

end

end