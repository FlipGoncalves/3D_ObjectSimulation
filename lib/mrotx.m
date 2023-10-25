function [M] = mrotx(av)
% Returns hipermatrix of rotations around X axis
% av - vector of rotation values

M = zeros(4,4,length(av));
for i = 1:length(av)
    a = av(i);
    M(:,:,i) = [1 0 0 0
                0 cos(a) -sin(a) 0
                0 sin(a) cos(a) 0
                0 0 0 1];
end
end