function quat = EulerToQuaternion(roll,pitch,yaw)
    quat = DCMToQuaternion(EulerToDCM(roll,pitch,yaw));
end