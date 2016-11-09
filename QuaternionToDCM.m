function DCM = QuaternionToDCM(quat)

DCM = zeros(3,3);
DCM(1,1) = quat(1) * quat(1) + quat(2) * quat(2) - quat(3) * quat(3) - quat(4) * quat(4);
DCM(1,2) = 2 * (quat(2) * quat(3) + quat(1) * quat(4));
DCM(1,3) = 2 * (quat(2) * quat(4) - quat(1) * quat(3));

DCM(2,1) = 2 * (quat(2) * quat(3) - quat(1) * quat(4));
DCM(2,2) = quat(1) * quat(1) - quat(2) * quat(2) + quat(3) * quat(3) - quat(4) * quat(4);
DCM(2,3) = 2 * (quat(3) * quat(4) + quat(1) * quat(2));

DCM(3,1) = 2 * (quat(2) * quat(4) + quat(1) * quat(3));
DCM(3,2) = 2 * (quat(3) * quat(4) - quat(1) * quat(2));
DCM(3,3) = quat(1) * quat(1) - quat(2) * quat(2) - quat(3) * quat(3) + quat(4) * quat(4);
end