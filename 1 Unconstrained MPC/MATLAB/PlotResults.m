% This script is automatically called at the end of the 
% Simulink simulation to plot the results (using the StopFcn 
% callback). The data is extracted the enabled logging of 
% the needed variables.
% This script can be run independantly, but only after the 
% Simulink simulation has been run at least once (so that the
% variables are available in the Workspace).

y = out.logsout{1}.Values.Data;
yk = out.logsout{2}.Values.Data;
uk = out.logsout{7}.Values.Data;

t = out.tout;
figure(9); % corresponds with figure number in the report
stairs(t, Ysp', '--')
hold on
% stairs(t,uk)
plot(t, y)
stairs(t, yk)
hold off
title('Simulink Simulation Result');
xlabel('Simulink Discrete Time k');
ylim([-7,7]);
leg = legend('$Y_{sp1}(t)$', '$Y_{sp2}(t)$', ...
    '$y_1^*(t)$', '$y_2^*(t)$', ...
    '$y_1^*(k)$', '$y_2^*(k)$', ...
    'Location','east');
set(leg, 'Interpreter', 'latex');
set(gcf,'Position',[200 200 600 400])
matlab2tikz('Figures\Outputs.tex'); % library path added in SetupParameters script

figure(10); % corresponds with figure number in the report
stairs(t,uk)
title('Optimal Inputs');
xlabel('Simulink Discrete Time k');
leg = legend('$u_1^*(k)$', '$u_2^*(k)$', ...
    'Location','northeast');
set(leg, 'Interpreter', 'latex');
set(gcf,'Position',[200 200 600 400])
matlab2tikz('Figures\Inputs.tex');
