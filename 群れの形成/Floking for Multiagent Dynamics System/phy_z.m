%% 本論文で定義されたシグモイド関数
function phy = phy_z(z)
    global a b c
    
    phy = 0.5 * ((a + b) * omega_z(z + c) + (a - b));
end