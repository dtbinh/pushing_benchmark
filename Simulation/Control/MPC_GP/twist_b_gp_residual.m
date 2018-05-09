function twist_b = twist_b_gp_residual(gp_input)
    dx = twist_b_gp1_residual([gp_input]);
    dy = twist_b_gp2_residual([gp_input]);
    dtheta = twist_b_gp3_residual([gp_input]);
    twist_b = [dx;dy;dtheta];
end