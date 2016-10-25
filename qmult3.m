function q4 = qmult3(q1,q2,q3)
    q12 = qmult2(q1,q2);
    q4 = qmult2(q12,q3);