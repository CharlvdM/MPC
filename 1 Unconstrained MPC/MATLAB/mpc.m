function uOptimal = mpc(delayMod, sysVar, xk, storedData)
% This function implements the model predictive control (MPC)
% algorithm, which computes the optimal input steps to 
% minimizes the objective function.

objFunc = @(uFlat) ...
    objectiveFunc(uFlat, delayMod, sysVar, xk, storedData);

options = optimoptions('fminunc',...
    'Display','Iter',...
    'MaxFunEvals',Inf, 'MaxIterations',4000);

% take the last control implemented as the first guess of the
% new step
uGuess = storedData.uPrevious;
uGuess = reshape(uGuess,[],1); % flatten matrix

% find the optimal control steps
tic
uOptimal = fminunc(objFunc, uGuess, options);
toc

uOptimal = reshape(uOptimal,2,[]);

end