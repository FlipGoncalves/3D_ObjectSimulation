function MQ = LinspaceVect(Qi, Qf, N)
    % Applies a linspace operation to each vector element

    [rows, cols] = size(Qi);
    
    if rows > cols
        MQ = zeros(length(Qi), N);
        for i = 1:length(Qi)
            MQ(i,:) = linspace(Qi(i), Qf(i), N);
        end
    else
        MQ = zeros(N, length(Qi));
        for i = 1:length(Qi)
            MQ(:,i) = linspace(Qi(i), Qf(i), N);
        end
    end
end