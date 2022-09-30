function ddq = P_dynamics(mQ, q, dq, F)

Lddq = - q * ( L*In_pro(dq, dq) - In_pro(q, F)/mQ ) - F/mQ;

ddq = Lddq/L;