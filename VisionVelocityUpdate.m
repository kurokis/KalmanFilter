function [x_est,P_est] = VisionVelocityUpdate(x_pred,P_pred,vision,R_diag)

q = x_pred(1:4);
C = QuaternionToDCM(q);
v = x_pred(5:7);
measurement = vision;
predicted_measurement = C*v;
residual = measurement - predicted_measurement;
U = SkewSymmetricFromVector3(predicted_measurement);
H = [U C zeros(3)];
R = diag(R_diag);

S = H*P_pred*H'+R;
K = P_pred*H'*inv(S);
P_est = (eye(9)-K*H)*P_pred;
dx = K*residual;
x_est = MapErrorToFullStates(x_pred,dx);

end
