%% 関数phyとlo_hの積
function phy_alpha = phy_alpha_z(z)
    global r_alpha d_alpha;
    phy_alpha = lo_h(z/r_alpha) * phy_z(z - d_alpha);
end
