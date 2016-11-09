function [x_ests, P_ests, ts, data] = KF(filename,heading_xaxis)
    % filename: filename of csv with sensor data. 'signal.csv'
    % heading_xaxis: initial heading in NED-frame.
    % The unit is in radian starting from North and increasing in the
    % clockwise direction. (North=>0, East=>90*pi/180, etc.)
    
    M = csvread(filename);
    data = []; % for exporting miscellaneous data
    
    % parameters
    dt = 1/128; % IMU sampling interval (s)
    gps_home = [0; 0; 0]; % longitude,latitude,height
    accelerometer_sigma = 10.*[0.005 0.005 0.042]; % standard deviation (m/s^2)
    gyro_sigma = [0.007 0.007 0.007]; % standard deviation (rad/s)
    vision_sigma = [0.02 0.02 0.02]; % standard deviation (m/s)
    usevision = 1; % whether to use vision velocity in measurement update
    usegps = 1; % whether to use GPS position in measurement update
    calibrate = ~strcmp(filename,'simulator.csv'); % do not calibrate if using simulator data
    sampling_duration = 10; % sampling duration for calibrating IMU
        
    % initialize
    N = sum(M(:,1) == 0);
    ts = linspace(0,dt*N,N+1);
    x_ests = zeros(10,N+1);
    P_ests = zeros(9,9,N+1);
    x_ests(:,1) = [1;0;0;0;0;0;0;0;0;0]; % [q0;q1;q2;q3;vx;vy;vz;rx;ry;rz]
    P_ests(:,:,1) = 0.00001.*eye(9);
    if(calibrate)
        [accelerometer_bias,gyro_bias] = CalibrateIMU(filename,sampling_duration,dt);
    else
        accelerometer_bias = [0;0;0];
        gyro_bias = [0;0;0];
    end
    
    % loop
    firstgps = 1;
    j=2; % j: index for storing data
    for i = 1:size(M,1) % i: index for scanning input data
        switch M(i,1)
            case 0
                % IMU
                x_pv = x_ests(:,j-1); % past value
                P_pv = P_ests(:,:,j-1); % past value
                accelerometer = reshape(M(i,3:5),[3 1]).*(5/1024/8*9.8); % (m/s^2)
                gyro = reshape(M(i,6:8),[3 1]).*(5/6.144/8*0.01745); % (rad/s)
            
                gyro = gyro - gyro_bias;
                accelerometer = accelerometer - accelerometer_bias;
                
                [x_pred,P_pred] = TimeUpdate(x_pv,P_pv,accelerometer,gyro,accelerometer_sigma.^2,gyro_sigma.^2,dt); % current prediction
                [x_est,P_est] = AccelerometerUpdate(x_pred,P_pred,accelerometer,accelerometer_sigma.^2); % current estimate
                
                x_ests(:,j) = x_est; % current estimate
                P_ests(:,:,j) = P_est; % current estimate
                j = j+1; % advance timestep
            case 8
                % Vision Velocity
                if(usevision)
                    x_pred = x_ests(:,j-1); % current estimate
                    P_pred = P_ests(:,:,j-1); % current estimate
                    vision = reshape(M(i,6:8),[3 1]).*(0.001 * 30); % (m/s)

                    [x_est,P_est] = VisionVelocityUpdate(x_pred,P_pred,vision,vision_sigma.^2);

                    x_ests(:,j-1) = x_est; % overwrite with new estimate
                    P_ests(:,:,j-1) = P_est; % overwrite with new estimate
                end
            case 1
                % GPS Position
                if(usegps)
                    x_pred = x_ests(:,j-1); % current estimate
                    P_pred = P_ests(:,:,j-1); % current estimate

                    %gps_now = reshape(M(i,4:6),[3 1]); % longitude,latitude,height_mean_sea_level
                    longitude_em7d = M(i,4); % longitude (10^-7 deg)
                    latitude_em7d = M(i,5); % latitude (10^-7 deg)
                    height_em3m = M(i,6); % height above mean sea level (10^-3 m)
                    horizontal_accuracy_em3m = M(i,7); % horizontal accuracy (10^-3 m)
                    vertical_accuracy_em3m = M(i,8); % vertical accuracy (10^-3 m)
                    gps_position = struct( ...
                        'longitude_em7d',longitude_em7d, ...
                        'latitude_em7d',latitude_em7d, ...
                        'height_em3m',height_em3m, ...
                        'horizontal_accuracy_em3m',horizontal_accuracy_em3m, ...
                        'vertical_accuracy_em3m',vertical_accuracy_em3m);
                    if firstgps==1
                        latitude_rad = (latitude_em7d/10^7)*pi/180;
                        longitude_scale = 10^(-7)*(111132.954 * cos(latitude_rad)); % WGS84 spheroid
                        latitude_scale = 10^(-7)*(111132.954 - 559.822 * cos(2*latitude_rad) + 1.175 * cos(4*latitude_rad)); % WGS84 spheroid
                        height_scale = 0.001;
                        accuracy_scale = 0.001;
                        gps_home = struct( ...
                            'longitude_em7d',longitude_em7d, ...
                            'latitude_em7d',latitude_em7d, ...
                            'height_em3m',height_em3m, ...
                            'longitude_scale',longitude_scale, ...
                            'latitude_scale',latitude_scale, ...
                            'height_scale',height_scale, ...
                            'accuracy_scale',accuracy_scale, ...
                            'heading_xaxis',heading_xaxis);
                        firstgps = 0;
                    end
                    
                    [x_est,P_est,GPSposition] = GPSPositionUpdate(x_pred,P_pred,gps_position,gps_home);

                    x_ests(:,j-1) = x_est; % overwrite with new estimate
                    P_ests(:,:,j-1) = P_est; % overwrite with new estimate
                    
                    data = [data;GPSposition];
                end
            otherwise
                % pass
        end
    end
end