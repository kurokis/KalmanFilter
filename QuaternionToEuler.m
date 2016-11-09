function [roll,pitch,yaw] = QuaternionToEuler(quat)
    [roll,pitch,yaw] = DCMToEuler(QuaternionToDCM(quat));
end