function [M] = mrotz(av)
% Returns hipermatrix of rotations around Z axis
% av - vector of rotation values in radians

M = zeros(4,4,length(av));
for i = 1:length(av)
    a = av(i);
    M(:,:,i) = [cos(a) -sin(a) 0 0
                sin(a) cos(a) 0 0
                0 0 1 0
                0 0 0 1];
end
end