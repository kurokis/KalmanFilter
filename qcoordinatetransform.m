function vb = qcoordinatetransform(vi,qib)
    vb_quat = qmult3(qconj(qib),[0;vi],qib);
    vb = vb_quat(2:4);
