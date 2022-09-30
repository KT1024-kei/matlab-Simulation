%% phy_alphaの積分
function phy_alpha2 = phy_alpha_z2(z)
    global d_alpha
    syms x;
    
    f = phy_alpha_z(x);
    phy_alpha2 = int(f, x, [d_alpha z]);

end