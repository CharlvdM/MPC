% This script is automatically called at the end of the 
% Simulink simulation to plot the results (using the StopFcn 
% callback). The data is extracted the enabled logging of 
% the needed variables.
% This script can be run independantly, but only after the 
% Simulink simulation has been run at least once (so that the
% variables are available in the Workspace).

yk = out.logsout{1}.Values.Data;
uk = out.logsout{2}.Values.Data;
dk = out.logsout{3}.Values.Data;
dkbar = out.logsout{4}.Values.Data;
y = out.logsout{5}.Values.Data;

t = out.tout;
figure(6); % corresponds with figure number in the report
stairs(t, YspArr', '--')
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
    '$\hat{y}_1(k)$', '$\hat{y}_2(k)$', ...
    'Location','east');
set(leg, 'Interpreter', 'latex');
set(gcf,'Position',[200 200 600 400])
if savePlotData == true
    matlab2tikz('Figures\Outputs.tex'); 
        % library path added in SetupParameters script
end

figure(7); % corresponds with figure number in the report
stairs(t,uk)
title('Optimal Inputs');
xlabel('Simulink Discrete Time k');
leg = legend('$u_1^*(k)$', '$u_2^*(k)$', ...
    'Location','northeast');
set(leg, 'Interpreter', 'latex');
set(gcf,'Position',[200 200 600 400])
if savePlotData == true
    matlab2tikz('Figures\Inputs.tex');
end

figure(5); % corresponds with figure number in the report
stairs(t,dk)
hold on
stairs(t,dkbar)
hold off
title('Disturbance Filter');
xlabel('Simulink Discrete Time k');
leg = legend('$d_1(k)$', '$d_2(k)$', ...
    '$\bar{d}_1(k)$', '$\bar{d}_2(k)$', ...
    'Location','east');
set(leg, 'Interpreter', 'latex');
set(gcf,'Position',[200 200 600 400])
if savePlotData == true
    matlab2tikz('Figures\Disturbance.tex');
end
