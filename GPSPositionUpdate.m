function [x_est,P_est,GPSposition] = GPSPositionUpdate(x_pred,P_pred,gps_position,gps_home)
    % --- GPS specific operations ---
    x_ned = gps_home.latitude_scale*(gps_position.latitude_em7d - gps_home.latitude_em7d);
    y_ned = gps_home.longitude_scale*(gps_position.longitude_em7d - gps_home.longitude_em7d);
    z_ned = gps_home.height_scale*(gps_position.height_em3m - gps_home.height_em3m);
    position = [cos(gps_home.heading_xaxis) * x_ned + sin(gps_home.heading_xaxis) * y_ned;
        -sin(gps_home.heading_xaxis) * x_ned + cos(gps_home.heading_xaxis) * y_ned;
        z_ned];
    scale = 5.0; % a parameter to adjust confidence in GPS position data
    h_sigma = scale*gps_home.accuracy_scale*gps_position.horizontal_accuracy_em3m; % standard deviation (m)
    v_sigma = scale*gps_home.accuracy_scale*gps_position.vertical_accuracy_em3m; % standard deviation (m)
    R_diag = [h_sigma h_sigma v_sigma].^2;
    % --- End of GPS specific operations ---

    r = x_pred(8:10);
    measurement = position;
    predicted_measurement = r;
    residual = measurement - predicted_measurement;
    H = [zeros(3,6) eye(3)];
    R = diag(R_diag);

    S = H*P_pred*H'+R;
    K = P_pred*H'*inv(S);
    P_est = (eye(9)-K*H)*P_pred;
    dx = K*residual;
    x_est = MapErrorToFullStates(x_pred,dx);

    GPSposition = position';
end
