function [C, Ceq] = getConstraints(uFlat, sysVar)

Nc = sysVar.Nc; % number of control steps
p = sysVar.p;   % number of inputs, p=2

uControl = reshape(uFlat,2,[]);

C = zeros(p,Nc);

for i = 1:p
    for j = 1:Nc
        if i == 1
            C(1,j) = uControl(1,j)-1;
        else
            C(2,j) = -uControl(2,j)-4;
        end
    end
end

C = reshape(C,[],1);

% C = [];
% this method is less preferred, because the 
% size of C changes
% for i = 1:Nc 
%     C = [C; uControl(1,i)-1; -uControl(2,i)-4];
% end

Ceq = [];
end