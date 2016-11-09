function x_est = MapErrorToFullStates(x_pred,dx)

q = x_pred(1:4);
alpha = dx(1:3);
Psi=[-q(2) -q(3) -q(4); q(1) -q(4)  q(3); q(4) q(1) -q(2); -q(3) q(2) q(1)];
dq = 0.5*Psi*alpha;
x_est = x_pred+[dq;dx(4:end)];
x_est(1:4) = x_est(1:4)./norm(x_est(1:4));

end