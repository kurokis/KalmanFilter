function quat = DCMToQuaternion(DCM)

tr = DCM(1,1)+DCM(2,2)+DCM(3,3);
if (tr>0)
    s = 1/(2*sqrt(1+tr));
    q0 = 1/(4*s); % scalar part
    q1 = (DCM(2,3)-DCM(3,2))*s;
    q2 = (DCM(3,1)-DCM(1,3))*s;
    q3 = (DCM(1,2)-DCM(2,1))*s;
elseif ( (DCM(1,1)>DCM(2,2)) && (DCM(1,1)>DCM(3,3)) )
    % C11 > C22,C33, (alpha,beta,gamma)=(1,2,3)
    s = 1/(2*sqrt(1+DCM(1,1)-DCM(2,2)-DCM(3,3)));
    q1 = 1/(4*s);
    q2 = (DCM(1,2)+DCM(2,1))*s;
    q3 = (DCM(3,1)+DCM(1,3))*s;
    q0 = (DCM(2,3)-DCM(3,2))*s;
elseif (DCM(2,2)>DCM(3,3))
    % C22 >= C11,C33, (alpha,beta,gamma)=(2,3,1)
    s = 1/(2*sqrt(1+DCM(2,2)-DCM(3,3)-DCM(1,1)));
    q2 = 1/(4*s);
    q3 = (DCM(2,3)+DCM(3,2))*s;
    q1 = (DCM(1,2)+DCM(2,1))*s;
    q0 = (DCM(3,1)-DCM(1,3))*s;
else
    % C33 >= C11,C22, (alpha,beta,gamma)=(3,1,2)
    s = 1/(2*sqrt(1+DCM(3,3)-DCM(1,1)-DCM(2,2)));
    q3 = 1/(4*s);
    q1 = (DCM(3,1)+DCM(1,3))*s;
    q2 = (DCM(2,3)+DCM(3,2))*s;
    q0 = (DCM(1,2)-DCM(2,1))*s;
end

quat = [q0;q1;q2;q3];
end