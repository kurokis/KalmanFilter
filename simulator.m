% parameters
dt = 1/128;
timesteps = 128;
quaternion_parameters = [0.5 pi 0; 0 0 0; 0.2 pi 0]; % col1:amp, col2:freq, bias
position_parameters = [0*1 0*pi; 0 0; 0 0]; % col1:amp, col2:freq
gps_home = (10^7).*[139; 39]; % longitude,latitude
heading_xaxis = 90*pi/180; % initial heading in NED-frame
accelerometer_sigma = [0.005 0.005 0.042];
gyro_sigma = [0.007 0.007 0.007];
vision_sigma = [0.02 0.02 0.02];
gpspos_sigma = [5 5 5];
gpsvel_sigma = [0.7 0.7 0.7];
gpscourse_sigma = 40*10^5;
gravity = 9.8;
export_filename = 'signal.csv';

% initialization
delete('signal.csv');
ts = linspace(0,dt*(timesteps-1),timesteps);
xs = zeros(10,timesteps);
a = quaternion_parameters(1,1);b = quaternion_parameters(1,2);bx = quaternion_parameters(1,3);
c = quaternion_parameters(2,1);d = quaternion_parameters(2,2);by = quaternion_parameters(2,3);
e = quaternion_parameters(3,1);f = quaternion_parameters(3,2);bz = quaternion_parameters(3,3);
qs_vectorpart = [a*sin(b.*ts)+bx; c*sin(d.*ts)+by; e*sin(f.*ts)+bz];
qs_scalarpart = sqrt(1-qs_vectorpart(1,:).^2-qs_vectorpart(2,:).^2-qs_vectorpart(3,:).^2);
xs(1:4,:) = [qs_scalarpart;qs_vectorpart];
a = position_parameters(1,1);b = position_parameters(1,2);
c = position_parameters(2,1);d = position_parameters(2,2);
e = position_parameters(3,1);f = position_parameters(3,2);
xs(5:7,:) = [a*b*cos(b.*ts); c*d*cos(d.*ts); e*f*cos(f.*ts)];
xs(8:10,:) = [a*sin(b.*ts); c*sin(d.*ts); e*sin(f.*ts)];

lon_scale = 0.0111132954 * cos(gps_home(2)/(10^7)*pi/180);
lat_scale = 0.0111132954 - 0.0000559822 * cos(2*gps_home(2)/(10^7)*pi/180) + 0.0000001175 * cos(4*gps_home(2)/(10^7)*pi/180);
  
timestamps = zeros(1,timesteps);
wbs = zeros(3,timesteps); % true angular velocity in b-frame
gyros = zeros(3,timesteps); % angular velocity in b-frame with noise
fbs = zeros(3,timesteps); % true specific force in b-frame
accelerometers = zeros(3,timesteps); % specific force in b-frame with noise
vbs = zeros(3,timesteps); % true velocity in b-frame
visions = zeros(3,timesteps); % velocity in b-frame with noise
gpsposs = zeros(3,timesteps); % position in NED-frame
gpsvels = zeros(3,timesteps); % velocity in NED-frame
gpscourses = zeros(1,timesteps); % course in NED-frame

% ------ Generate sensor data -------
for i = 2:timesteps
    t = ts(i);
    q_pv = xs(1:4,i-1);
    q = xs(1:4,i);
    c_heading = [cos(heading_xaxis) -sin(heading_xaxis); sin(heading_xaxis) cos(heading_xaxis)];
    
    % ------ Timestamp -------
    timestamps(:,i) = floor(t*1000);
    
    % ------ Accelerometer -------
    fbs(:,i) = qcoordinatetransform([0;0;-gravity],q);
    accelerometers(:,i) = floor((fbs(:,i) + diag(accelerometer_sigma)*randn([3 1]))/(5/1024/8*9.8));
    
    % ------ Gyro -------
    % This method is not computationally efficient.
    %dq = qmult2(qconj(q_pv),q);
    %n = dq(2:4)./norm(dq(2:4)); % unit rotation vector in the frame of q_pv (and also q)
    %dtheta = 2*asin( norm(dq(2:4)) );
    %wb = n.*(dtheta/dt);
    
    % This method is more computationally efficient.
    dq = qmult2(qconj(q_pv),q);
    wb_quat = (2/dt).*qmult2(qconj(q_pv),(q-q_pv));
    wbs(:,i) = wb_quat(2:4);
    gyros(:,i) = floor((wbs(:,i) + diag(gyro_sigma)*randn([3 1]))/(5/6.144/8*0.01745));
    
    % ------ Vision -------
    vbs(:,i) = qcoordinatetransform(xs(5:7,i),q);
    visions(:,i) = (vbs(:,i) + diag(vision_sigma)*randn([3 1]))/(0.001 * 30); % mm/frame
    
    % ------ GPS Position -------
    gpspos_i = xs(8:10,i)+diag(gpspos_sigma)*randn([3 1]); % i-frame
    gpspos_ne = c_heading*gpspos_i(1:2); % NE(D)-frame
    gpsposs(:,i) = [floor(gps_home(1)+gpspos_ne(1)/lon_scale); % lon * 10^7
                    floor(gps_home(2)+gpspos_ne(2)/lat_scale); % lat * 10^7
                    floor(gpspos_i(3)*1000)];                  % height in meters * 10^3
    
    % ------ GPS Velocity -------
    gpsvel_ned = [c_heading*xs(5:6,i);xs(7,i)]+diag(gpsvel_sigma)*randn([3 1]);
    gpsvels(:,i) = floor(100.*gpsvel_ned);
    gpscourses(:,i) = floor(atan2(gpsvel_ned(1),gpsvel_ned(2))*180/pi*10^5);
    
end

% ------ Export sensor data -------
for i = 2:timesteps
    timestamp = timestamps(:,i);
    accelerometer = accelerometers(:,i);
    gyro = gyros(:,i);
    vision = visions(:,i);
    gpspos = gpsposs(:,i);
    gpsvel = gpsvels(:,i);
    gpscourse = gpscourses(:,i);
    
    % ------ FlightCtrl -------
    row = [0 timestamp accelerometer' gyro' -1 -1];
    dlmwrite(export_filename,row,'-append');
    
    % ------ Vision ------
    row = [8 timestamp -1 -1 1 vision' [-1;-1;-1]' [-1;-1;-1]' [-1;-1;-1]' -1 -1 -1 -1 -1 -1 -1];
    dlmwrite(export_filename,row,'-append');
    
    % ------ GPS Position ------
    row = [1 timestamp -1 gpspos' floor(gpspos_sigma(1)*1000) floor(gpspos_sigma(3)*1000)];
    dlmwrite(export_filename,row,'-append');
    
    % ------ GPS Velocity ------
    row = [2 timestamp -1 gpsvel' -1 -1 gpscourse gpsvel_sigma(1) gpscourse_sigma];
    dlmwrite(export_filename,row,'-append');
    
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

%P0_diag = 0.00001.*[1 1 1 1 1 1 1 1 1];
%Ps = zeros(9,9,timesteps);
%Ps(:,:,1) = diag(P0_diag);
