function [x_pred,P_pred] = TimeUpdate(x_est,P_est,accelerometer,gyro, accelerometer_sigma, gyro_sigma, dt)
    q = x_est(1:4);
    v = x_est(5:7);
    r = x_est(8:10);
    
    % ------ update x ------
    %qdot =[ 0.5*( - gyro(1)*q(2) - gyro(2)*q(3) - gyro(3)*q(4));
    %        0.5*( + gyro(1)*q(1) + gyro(3)*q(3) - gyro(2)*q(4));
    %        0.5*( + gyro(2)*q(1) - gyro(3)*q(2) + gyro(1)*q(4));
    %        0.5*( + gyro(3)*q(1) + gyro(2)*q(2) - gyro(1)*q(3))];   
    qdot = 0.5*qmult2(q,[0;gyro]);
    q_next_ = q+qdot.*dt;
    q_next = q_next_./norm(q_next_);
    vdot = qcoordinatetransform(accelerometer,qconj(q))+[0; 0; 9.8];
    rdot = v;
    v_next = v+vdot.*dt;
    r_next = r+rdot.*dt;
    x_pred = [q_next;v_next;r_next];
    
    %I = eye(3);
    %O = zeros(3);
    %A = I-dt.*SkewSymmetricFromVector3(gyro);
    %B = -QuaternionToDCM(qconj(q))*(dt.*SkewSymmetricFromVector3(accelerometer));
    %C = dt.*I;
    %Phi = [A O O; B I O; O C I];
    %D = -dt.*QuaternionToDCM(qconj(q));
    %Gamma = [-C O; O D; O O];
    
    A = [-SkewSymmetricFromVector3(gyro) zeros(3) zeros(3);
        -QuaternionToDCM(qconj(q))*SkewSymmetricFromVector3(accelerometer) zeros(3) zeros(3);
        zeros(3) eye(3) zeros(3)];
    Phi = eye(9)+A.*dt;
    B = [eye(3) zeros(3);zeros(3) eye(3); zeros(3) zeros(3)];
    Gamma = B.*dt;
    
    Q = [diag(gyro_sigma) zeros(3); zeros(3) diag(accelerometer_sigma)];
    
    Qprime = diag([0 0 0 0.01 0.01 0.01 0.001 0.001 0.001]);
    
    P_pred = Phi*P_est*Phi'+Gamma*Q*Gamma'+Qprime;