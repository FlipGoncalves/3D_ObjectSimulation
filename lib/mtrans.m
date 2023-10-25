function [M] = mtrans(X, Y, Z)
% Returns hipermatrix of translations
% X - vector of X axis translations
% Y - vector of Y axis translations
% Z - vector of Z axis translations

m = max([numel(X), numel(Y), numel(Z)]);
X(end:m) = X(end); Y(end:m) = Y(end); Z(end:m) = Z(end);

M = zeros(4,4,m);
for i = 1:m
    M(:,:,i) = [1 0 0 X(i)
                0 1 0 Y(i)
                0 0 1 Z(i)
                0 0 0 1];
end
end