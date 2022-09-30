%% 隆起関数を計算する関数
function loh = lo_h(z)
    global h;
    
    if 0 <= z && z < h
        loh = 1;
    elseif h <= z && z <= 1
        loh = 0.5 * (1 + cos(pi * (z - h)/(1 - h)));
    else
        loh = 0;
    end
end