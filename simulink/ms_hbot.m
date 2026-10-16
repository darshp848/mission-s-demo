function [h_bot, xmin, xmax] = ms_hbot(P, pos, q, cg)
%MS_HBOT  Lowest body-box corner height and x extent in the world frame.
R = ms_rot(q);
box_zc = -P.h_ref + 0.10 + P.H/2;
h_bot = inf; xmin = inf; xmax = -inf;
for sx = [-1 1]
    for sy = [-1 1]
        for sz = [-1 1]
            c = [sx*P.L/2 - cg(1); sy*P.W/2 - cg(2); box_zc + sz*P.H/2 - cg(3)];
            w = R*c;
            h_bot = min(h_bot, pos(3) + w(3));
            xmin = min(xmin, pos(1) + w(1));
            xmax = max(xmax, pos(1) + w(1));
        end
    end
end
end
