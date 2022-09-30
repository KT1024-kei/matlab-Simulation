function ddP = Q_dynamics(mQ, mL, q, dq, F, G)

mQddP = mL/(mQ + mL) * q * ( mQ*L*In_pro(dq, dq) - In_pro(q, F) ) + mQ*G + F;

ddP = mQddP/mQ;