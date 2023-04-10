function cost = ...
    objectiveFunc(uFlat, delayMod, sysVar, xk, storedData)

% Extract needed system variables
Np = sysVar.Np;
Nc = sysVar.Nc;
Q = sysVar.Q;
R = sysVar.R;

% predict the future plant behaviour based on iputs uControl
uControl = reshape(uFlat,2,[]);
[yk, uk, z1] = ...
    prediction(uControl, delayMod, sysVar, xk, storedData);

% Current setpoint
Ysp = delayMod.Ysp;

%% Calculate objective function
cost = 0;

% Output cost contribution
for i = 1:Np
    cost = cost + (Ysp-yk(:,i))'*Q*(Ysp-yk(:,i));
end

% Input cost contribution
uDel = zeros(2, Nc);
u0 = storedData.uPrevious(:,1); % last implemented u

uDel(:,1) = uControl(:,1) - u0;
for i = 2:Nc
    uDel(:,i) = uControl(:,i) - uControl(:,i-1);
end

for i = 1:Nc
    cost = cost + uDel(:,i)'*R*uDel(:,i);
end

end