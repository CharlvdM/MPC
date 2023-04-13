function uOptimal = ...
    mpc(plantD, Ysp, sysVar, xk, dk, uPrevious)
% This function implements the model predictive control (MPC)
% algorithm, which computes the optimal input steps to 
% minimizes the objective function.

objFunc = @(uFlat) objectiveFunc(...
    uFlat, plantD, Ysp, sysVar, xk, dk, uPrevious);

nonLinCon = @(uFlat) getConstraints(uFlat, sysVar);

% options = optimoptions('fminunc',...
%     'Display','Iter',...
%     'MaxFunEvals',Inf, 'MaxIterations',4000);

% options = optimoptions('fmincon','Display','Iter',...
%     'Algorithm','sqp',...
%     'MaxFunEvals',Inf, 'MaxIterations',4000);

options = optimoptions('fmincon','Display','Iter',...
    'MaxFunEvals',Inf, 'MaxIterations',4000);

% take the last control implemented as the first guess of the
% new step
uGuess = uPrevious;
uGuess = reshape(uGuess,[],1); % flatten matrix

% find the optimal constrained control steps
tic
% uOptimal = fminunc(objFunc, uGuess, options);

% x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon) subjects the 
% minimization to the nonlinear inequalities c(x) or 
% equalities ceq(x) defined in nonlcon. fmincon optimizes 
% such that c(x) <= 0 and ceq(x) = 0.
uOptimal = fmincon(objFunc, uGuess,...
    [],[],[],[],[],[], nonLinCon, options);
toc

uOptimal = reshape(uOptimal,2,[]);

end