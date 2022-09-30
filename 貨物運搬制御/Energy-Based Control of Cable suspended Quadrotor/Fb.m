function fb = Fb(mL, mQ, kQ, kL, ga)

fb = -1/kQ * ( eb2 + eb1 + (kQ*mQ + kL*mL) * ga);