close;
clear;

% Check if the file exists in the current directory
fileToCheck = 'traj.csv';
if exist(fileToCheck, 'file') ~= 2
    disp(['File "', fileToCheck, '" does not exist in the current directory!']);
    
    % default trajectory
    solution = [ 5	 5	 5	1.3   -45	1
                 5	 2	 1	  0	    0	1
                 3	 2	-4	1.1	   30	0
                -2	 4	 1	1.2  -160	0
                 2	 3	 1	  0	    0	0
                -3	 1	 2	1.5	   90	1
                -2	-1	-3	1.4	  -30	0
                -2	-1	-2	0.9	  120	0
                -3	-3	 0	  0     0	1
                -4	-5	 1	1.2	   90	0
                 0	-4	 1	0.5	  -90	1
                 1	-3	-3	  2	   60	0 ];
else
    % read trajectory csv
    solution = csvread(fileToCheck);
end

% build referencial
axis equal;
axis([-2 20 -2 20 -2 20]);
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
view(120,30)
hold on;

numPyramids = 3;        % number of objects
N = 80;                % number of intermediate steps for each transformation
time_pause = 0.001;     % time between pauses (affects speed)

Points = cell(1, numPyramids);      % points of the objects
Faces = cell(1, numPyramids);       % faces of the objects

% create objects
Pir1 = Piramide(1);
Pir2 = Piramide(2);
Pir3 = Piramide(3);
[Points{1}, Faces{1}] = Pir1.getPointsFaces;
[Points{2}, Faces{2}] = Pir2.getPointsFaces;
[Points{3}, Faces{3}] = Pir3.getPointsFaces;
colors = {'y', 'g', 'b'};

% Store piece transformation matrices
transformations = zeros(4, 4, numPyramids);
H = cell(1, numPyramids);

% initial position
for i = 1:numPyramids
    transformations(:,:,i) = trans(0,0,0);
    p = transformations(:,:,i)*Points{i};
    
    H{i} = patch('Vertices', Points{i}(1:3,:)', ...
                 'Faces', Faces{i}, ...
                 'FaceColor', colors{i});
end

% transformations matrix and animated points
T = zeros([4 4 N 1]);
PointsAnimated = Points;

% animation cell for each object
animation = cell(1, numPyramids);

% draw path for the objects
colors = ["r", "g", "b", "c", "m", "y", "k"];
Pstart = [0 0 0];
mov = [0 0 0];
for i = 1:size(solution,1)
    movement = solution(i, :);
    h = movement(4);
    beta = movement(5) * pi / 180;
    Pfinish = [Pstart(1)+movement(1) Pstart(2)+movement(2) Pstart(3)+movement(3)];
    newpoint = [movement(1)+mov(1) movement(2)+mov(2) movement(3)+mov(3)];
    if h ~= 0
        P1 = [0 0 0];
        P2 = [movement(1) movement(2) movement(3)];
        midpoint = (P1 + P2) / 2;
        
        vector = (P2 - P1);
        vectorXY = [0 0 1];
        
        theta = atan2(vector(2), vector(1));
        normalized = vector/norm(vector);
        angleVector = acos(dot(normalized, vectorXY) / (norm(normalized)*norm(vectorXY)));
        phi = angleVector-pi/2;
        
        P4 = zeros(1, 3);
        P4(1) = midpoint(1) + h * sin(phi) * cos(theta);
        P4(2) = midpoint(2) + h * sin(phi) * sin(theta);
        P4(3) = midpoint(3) + h * cos(phi);
        
        [r, C, al, X, Y] = arc3(P1, P4, P2);
       
        vector1 = P1 - C;
        vector2 = [0 0 1];
        vector3 = P2 - C;
        
        angle_radians1 = atan2(norm(cross(vector1, vector2)), dot(vector1, vector2));
        angle_radians2 = atan2(norm(cross(vector3, vector2)), dot(vector3, vector2));
        steps_phi = linspace(angle_radians1, angle_radians1-al, N);
    
        Ptemp = zeros(3,1);
        Ptemp(1) = r * sin(steps_phi(1)) * cos(theta) + C(1);
        Ptemp(2) = r * sin(steps_phi(1)) * sin(theta) + C(2);
        Ptemp(3) = r * cos(steps_phi(1)) + C(3);
        if norm(P1-Ptemp') > 0.001
            steps_phi = linspace(-angle_radians1, -angle_radians1-al, N);
        else 
            if angle_radians1 > angle_radians2
                steps_phi = linspace(-angle_radians1, -angle_radians1-al, N);
            end
        end
    
        Raxis = axang2rotm([vector, beta]);
        Pst = Pstart;
        for ang=1:N
            P = zeros(3,1);
            P(1) = r * sin(steps_phi(ang)) * cos(theta) + C(1);
            P(2) = r * sin(steps_phi(ang)) * sin(theta) + C(2);
            P(3) = r * cos(steps_phi(ang)) + C(3);
            P = Raxis * P + mov';
            plot3([Pst(1), P(1)]', [Pst(2), P(2)]', [Pst(3), P(3)]', colors(mod(i, size(colors, 2))+1));
            Pst = P;
        end

        plot3([Pstart(1), Pfinish(1)]', [Pstart(2), Pfinish(2)]', [Pstart(3), Pfinish(3)]', colors(mod(i, size(colors, 2))+1), 'LineStyle','--');
    else
        plot3([Pstart(1), Pfinish(1)]', [Pstart(2), Pfinish(2)]', [Pstart(3), Pfinish(3)]', colors(mod(i, size(colors, 2))+1));
    end

    mov = newpoint;
    Pstart = Pfinish;
end

pause();

theta_last = 0;
phi_last = 0;

% beta
mov = [0, 0, 0];

% animate the object continuously
while true

    % build path
    for i = 1:size(solution,1)

        % next movement
        movement = solution(i, :);
    
        % h and beta
        h = movement(4);
        beta = movement(5) * pi / 180;
    
        % translations without beta and h
        x = linspace(0,movement(1),N);
        y = linspace(0,movement(2),N);
        z = linspace(0,movement(3),N);
        transx = [0 , linspace(x(2),x(2),N-1)];
        transy = [0 , linspace(y(2),y(2),N-1)];
        transz = [0 , linspace(z(2),z(2),N-1)];
    
        newpoint = [movement(1)+mov(1) movement(2)+mov(2) movement(3)+mov(3)];

        % translations with beta and h
        if h ~= 0

            % known two points in the circunference
            P1 = PointsAnimated{1}(1:end-1,1)';
            P2 = [P1(1)+movement(1) P1(2)+movement(2) P1(3)+movement(3)];
            
            % midpoint between P1 and P2
            midpoint = (P1 + P2) / 2;
            
            % vector and normal vector to XY plane
            vector = (P2 - P1);
            vectorXY = [0 0 1];
            
            % calculate theta (azimuthal angle) in radians
            theta = atan2(vector(2), vector(1));
            
            % calculate phi (polar angle) in radians
            normalized = vector/norm(vector);
            angleVector = acos(dot(normalized, vectorXY) / (norm(normalized)*norm(vectorXY)));
            phi = angleVector-pi/2;
            
            % calculate cartesian coordinates of the third point in the circunference
            P4 = zeros(1, 3);
            P4(1) = midpoint(1) + h * sin(phi) * cos(theta);
            P4(2) = midpoint(2) + h * sin(phi) * sin(theta);
            P4(3) = midpoint(3) + h * cos(phi);
            
            % get center, radius and arc angle
            [r, C, al, X, Y] = arc3(P1, P4, P2);
            
            % vectors from the points and the center
            vector1 = P1 - C;
            vector2 = [0 0 1];
            vector3 = P2 - C;
            
            % calculate angle the vectors do to the normal vector to XY plane
            angle_radians1 = atan2(norm(cross(vector1, vector2)), dot(vector1, vector2));
            angle_radians2 = atan2(norm(cross(vector3, vector2)), dot(vector3, vector2));
            
            % initiate arc angle from P1 to P2
            steps_phi = linspace(angle_radians1, angle_radians1-al, N);
        
            % calculate first point of arc
            Ptemp = zeros(3,1);
            Ptemp(1) = r * sin(steps_phi(1)) * cos(theta) + C(1);
            Ptemp(2) = r * sin(steps_phi(1)) * sin(theta) + C(2);
            Ptemp(3) = r * cos(steps_phi(1)) + C(3);
        
            % if the first point in the arc is not the same as P1 (by a slight margin) then change arc angle
            if norm(P1-Ptemp') > 0.001
                steps_phi = linspace(-angle_radians1, -angle_radians1-al, N);
            else 
                % if the first angle is greater than the second, change
                if angle_radians1 > angle_radians2
                    steps_phi = linspace(-angle_radians1, -angle_radians1-al, N);
                end
            end

            Raxis = axang2rotm([vector, beta]);
    
            % create points in the arc
            Parc = zeros(3, 4);
            for ang=1:N
                P = zeros(3,1);
                P(1) = r * sin(steps_phi(ang)) * cos(theta) + C(1);
                P(2) = r * sin(steps_phi(ang)) * sin(theta) + C(2);
                P(3) = r * cos(steps_phi(ang)) + C(3);
                P = Raxis * P + mov';
                Parc(:,ang) = P;
            end
    
            % get the differences between the points to do the translations
            [transx, transy, transz] = PointsFromArc(Parc);
        end
    
        % orientation
        [phi, theta, R] = cart2sph(movement(1), movement(2), movement(3));
        theta =  pi/2-theta;

        lin_theta_last = mroty(linspace(0, -theta_last, N));
        lin_phi_last = mrotz(linspace(0, -phi_last, N));
        lin_phi = mrotz(linspace(0, phi, N));
        lin_theta = mroty(linspace(0, theta, N));

        temp = ones(4,4,N);
        for step = 1:N
            temp(:,:,step) = lin_theta_last(:,:,step) * lin_phi_last(:,:,step) * lin_phi(:,:,step) * lin_theta(:,:,step);
        end

        % animate
        Ttemp = transformations(:,:,:);
        for step = 1:N
            for piece = 1:numPyramids
                Ttemp(:,:,piece) = transformations(:,:,piece)*temp(:,:,step);

                Pn = Ttemp(:,:,piece)*Points{piece};
                H{piece}.Vertices = Pn(1:3,:)';
                Pstart = Pn(:,4);
                PointsAnimated{piece} = Pn;
            end
            pause(time_pause);
        end
        transformations = Ttemp(:,:,:);

        % clear transformations matrix
        T = zeros([4 4 N 1]);

        T(:,:,:,1) = mtrans(transx,transy,transz);

        % see if there is twist movement
        if size(movement, 2) >= 6 && movement(6) ~= 0
            twirl = linspace(0,movement(6)*2*pi,N);
            T(:,:,:,2) = mrotz(linspace(twirl(2), twirl(2), N));

            for piece = 1:numPyramids
                % save transformation matrix
                animation{piece} = T;
            end
        else
            % singular movement for each pyramid
            for piece = 1:numPyramids
                trans_piece = zeros(N,1);
                transp = LinspaceVect([0 -0.5], [-0.5 0], N/2);
                for step = 2:N
                    trans_piece(step) = transp(step) - transp(step-1);
                end
        
                if piece == 1
                    T(:,:,:,2) = mtrans(0, trans_piece, 0);
                else
                    if piece == 2
                        T(:,:,:,2) = mtrans(-trans_piece, -trans_piece/2, 0);
                    else
                        T(:,:,:,2) = mtrans(trans_piece, -trans_piece/2, 0);
                    end
                end

                % save transformation matrix
                animation{piece} = T;
            end
        end

        % order
        order = [0 1 1];
    
        % animate
        Ttemp = transformations(:,:,:);
        for step = 1:size(animation{1}, 3)
            for transf = 1:size(animation{1}, 4)
                for piece = 1:numPyramids

                    if order(transf) 
                        Ttemp(:,:,piece) = transformations(:,:,piece)*animation{piece}(:,:,step,transf);
                    else
                        Ttemp(:,:,piece) = animation{piece}(:,:,step,transf)*transformations(:,:,piece);
                    end
    
                    Pn = Ttemp(:,:,piece)*Points{piece};

                    H{piece}.Vertices = Pn(1:3,:)';
                    Pstart = Pn(:,4);
                    PointsAnimated{piece} = Pn;
                end
                transformations = Ttemp(:,:,:);
            end
            pause(time_pause);
        end

        % orientation
        theta_last = theta;
        phi_last = phi;

        % beta
        mov = newpoint;
    end
end


