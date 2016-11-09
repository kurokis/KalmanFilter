function [roll,pitch,yaw] = DCMToEuler(DCM)
% Calculate euler angles from a DCM that denotes a rotation from inertial
% frame to body frame

pitch = asin(-DCM(1,3));
roll = atan2(DCM(2,3),DCM(3,3));
yaw = atan2(DCM(1,2),DCM(1,1));

end