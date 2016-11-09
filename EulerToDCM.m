function DCM = EulerToDCM(roll,pitch,yaw)
% Direction cosine matrix from inertial frame to body frame

DCM = [cos(pitch)*cos(yaw) cos(pitch)*sin(yaw) -sin(pitch);
       sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) sin(roll)*cos(pitch);
       cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw) cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw) cos(roll)*cos(pitch)];
end