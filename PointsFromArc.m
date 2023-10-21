function [transx,transy,transz] = PointsFromArc(Parc)

    N = size(Parc, 2);

    transx = zeros(N,1);
    transy = zeros(N,1);
    transz = zeros(N,1);

    for pt=2:N
        P1 = Parc(:,pt-1);
        P2 = Parc(:,pt);
        transx(pt) = P2(1) - P1(1);
        transy(pt) = P2(2) - P1(2);
        transz(pt) = P2(3) - P1(3);
    end
end

