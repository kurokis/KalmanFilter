function v2 = qrotatevector(v1,qib)
    v2_quat = qmult3(qib,[0;v1],qconj(qib));
    v2 = v2_quat(2:4);
