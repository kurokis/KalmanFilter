function [accelerometer_bias,gyro_bias] = CalibrateIMU(filename,sampling_duration,dt)
    % This function reads gyro value for given duration and uses its median as
    % the bias.

    M = csvread(filename);

    n = floor(sampling_duration/dt);
    gyros = zeros(n,3);
    accelerometers = zeros(n,3);

    i=1; % i: index for scanning input data
    j=1; % j: index for storing data
    while j<n+1
       if(M(i,1)==0) %IMU
           gyros(j,:) = reshape(M(i,6:8),[3 1]).*(5/6.144/8*0.01745); % (rad/s)
           accelerometers(j,:) = reshape(M(i,3:5),[3 1]).*(5/1024/8*9.8); % (m/s^2)
           j = j+1;
       end
       i = i+1;
    end

    accelerometer_bias = median(accelerometers)'+[0;0;9.8];
    gyro_bias = median(gyros)';
end