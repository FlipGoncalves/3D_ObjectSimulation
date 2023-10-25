function [M] = mroty(av)
% Returns hipermatrix of rotations around Y axis
% av - vector of rotation values in radians

M = zeros(4,4,length(av));
for i = 1:length(av)
    a = av(i);
    M(:,:,i) = [cos(a) 0 sin(a) 0
                0 1 0 0
                -sin(a) 0 cos(a) 0
                0 0 0 1];
end
end