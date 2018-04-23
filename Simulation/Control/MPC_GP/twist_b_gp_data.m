function twist_b = twist_b_gp_data(gp_input)
    dx = twist_b_gp1_data([gp_input]);
    dy = twist_b_gp2_data([gp_input]);
    dtheta = twist_b_gp3_data([gp_input]);
    twist_b = [dx;dy;dtheta];
end