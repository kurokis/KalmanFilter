% parameters
dt = 1/128;
timesteps = 128;
P0_diag = 0.00001.*[1 1 1 1 1 1 1 1 1];
quaternion_parameters = [0.1 2*pi; 0 0; 0.5 2*pi]; % col1:amp, col2:freq
position_parameters = [0*1 0*pi; 0 0; 0 0]; % col1:amp, col2:freq
GRAVITY = 9.8;

% initialization
ts = linspace(0,dt*(timesteps-1),timesteps);
xs = zeros(10,timesteps);
a = quaternion_parameters(1,1);b = quaternion_parameters(1,2);
c = quaternion_parameters(2,1);d = quaternion_parameters(2,2);
e = quaternion_parameters(3,1);f = quaternion_parameters(3,2);
qs_vectorpart = [a*sin(b.*ts); c*sin(d.*ts); e*sin(f.*ts)];
qs_scalarpart = sqrt(1-qs_vectorpart(1,:).^2-qs_vectorpart(2,:).^2-qs_vectorpart(3,:).^2);
xs(1:4,:) = [qs_scalarpart;qs_vectorpart];
a = position_parameters(1,1);b = position_parameters(1,2);
c = position_parameters(2,1);d = position_parameters(2,2);
e = position_parameters(3,1);f = position_parameters(3,2);
xs(5:7,:) = [a*b*cos(b.*ts); c*d*cos(d.*ts); e*f*cos(f.*ts)];
xs(8:10,:) = [a*sin(b.*ts); c*sin(d.*ts); e*sin(f.*ts)];

%Ps = zeros(9,9,timesteps);
%Ps(:,:,1) = diag(P0_diag);

wbs = zeros(3,timesteps); % true angular velocity in b-frame
gyros = zeros(3,timesteps); % angular velocity in b-frame with noise
fbs = zeros(3,timesteps); % true specific force in b-frame
accelerometers = zeros(3,timesteps); % specific force in b-frame with noise

% ------ Generate sensor data -------
for i = 2:timesteps
    t = ts(i);
    q_pv = xs(1:4,i-1);
    q = xs(1:4,i);
    dq = qmult2(qconj(q_pv),q);
    
    %n = dq(2:4)./norm(dq(2:4)); % unit rotation vector in the frame of q_pv (and also q)
    %dtheta = 2*asin( norm(dq(2:4)) );
    %wb = n.*(dtheta/dt);
    
    % This method is more computationally efficient.
    wb_quat = 2.*qmult2((q-q_pv)./dt,qconj(q_pv));
    wbs(:,i) = wb_quat(2:4);
    gyros(:,i) = wbs(:,i);
    
    fbs(:,i) = quatcoordinatetransform([0;0;-GRAVITY],q);
    accelerometers = fbs(:,i);
    
end

% ------ Double check for angular velocity calculation  -------
%qsdc = zeros(4,timesteps);
%qsdc(:,1) = [1;0;0;0];
%for i = 2:timesteps
%    q_pv = qsdc(:,i-1);
%    wc = wbs(:,i);
%    qdot = [ 0.5*( - wc(1)*q_pv(2) - wc(2)*q_pv(3) - wc(3)*q_pv(4));
%            0.5*( + wc(1)*q_pv(1) + wc(3)*q_pv(3) - wc(2)*q_pv(4));
%            0.5*( + wc(2)*q_pv(1) - wc(3)*q_pv(2) + wc(1)*q_pv(4));
%            0.5*( + wc(3)*q_pv(1) + wc(2)*q_pv(2) - wc(1)*q_pv(3))];
%    q_ = q_pv+qdot.*dt;
%    q = q_./norm(q_);
%    qsdc(:,i) = q;
%end
