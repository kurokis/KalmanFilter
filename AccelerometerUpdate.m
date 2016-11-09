function [x_est,P_est] = AccelerometerUpdate(x_pred,P_pred,accelerometer,R_diag)

q = x_pred(1:4);
C = QuaternionToDCM(q);
a0 = [0;0;-9.8];
measurement = accelerometer;
predicted_measurement = C*a0;
residual = measurement - predicted_measurement;
U = SkewSymmetricFromVector3(predicted_measurement);
H = [U zeros(3,6)];
R = diag(R_diag);

S = H*P_pred*H'+R;
K = P_pred*H'*inv(S);
P_est = (eye(9)-K*H)*P_pred;
dx = K*residual;
x_est = MapErrorToFullStates(x_pred,dx);

end