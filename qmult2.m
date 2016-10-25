function q3 = qmult2(q1,q2)
    q3s = q1(1)*q2(1)-dot(q1(2:4),q2(2:4));
    q3v = q1(1)*q2(2:4)+q2(1)*q1(2:4)+cross(q1(2:4),q2(2:4));
    q3 = [q3s;q3v];