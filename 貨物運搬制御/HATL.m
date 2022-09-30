function hatL = HATL(sigma, q, dPQ)

hatL = 1/SIGMA*Proj(sigma*In_pro(q, dPQ));
