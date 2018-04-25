function twist_b = twist_b_gp(vbpi, ry)
        switch nargin
            case 1
                dx = twist_b_gp1([vbpi]);
                dy = twist_b_gp2([vbpi]);
                dtheta = twist_b_gp3([vbpi]);
                twist_b = [dx;dy;dtheta];
            case 2
                dx = twist_b_gp1([vbpi;ry]);
                dy = twist_b_gp2([vbpi;ry]);
                dtheta = twist_b_gp3([vbpi;ry]);
                twist_b = [dx;dy;dtheta];
        end
end