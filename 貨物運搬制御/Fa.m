function fa = Fa(mL, mQ, Kq, Kl, ga, sigma, hatL, ea2, ea1)

fa = (mL + mQ)/(Kl*mL + Kq*mQ) * ...
     ( (Kq*mQ + Kl*mL) * ga + sigma*hatL + ea2 + ea1);
