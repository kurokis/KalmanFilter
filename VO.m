function [x_ests, ts, data] = VO(filename)
    M = csvread(filename);
    data = []; % for exporting miscellaneous data
    
    % parameters
    dt = 1/30;
    
    % initialize
    N = sum(M(:,1) == 8);
    ts = linspace(0,dt*N,N+1);
    x_ests = zeros(10,N+1);
    x_ests(:,1) = [1;0;0;0;0;0;0;0;0;0];
    
    % loop
    j=2; % j: index for storing data
    for i = 1:size(M,1) % i: index for scanning input data
        switch M(i,1)
            case 8
                % Vision
                x = x_ests(:,j-1); % previous

                vision_velocity = reshape(M(i,6:8),[3 1]).*(0.001 * 30); % mm/frame => m/s
                vision_angularrate = reshape(M(i,12:14),[3 1]).*(30); % rad/frame => rad/s
                
                data = [data; vision_velocity' vision_angularrate'];
                x_next = zeros(10,1);
                q = x(1:4);
                qdot = 0.5*qmult2(x(1:4),[0;vision_angularrate]);
                x_next(1:4) = x(1:4) + qdot.*dt;
                x_next(1:4) = x_next(1:4)./norm(x_next(1:4));
                vi = qcoordinatetransform(vision_velocity,qconj(q));
                x_next(5:7) = vi;
                x_next(8:10) = x(8:10) + vi.*dt;
                
                x_ests(:,j) = x_next;
                j = j+1;
            otherwise
                % pass
        end
    end
end