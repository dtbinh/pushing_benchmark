function k = covFunc(theta, xp, xq)
    ell = exp(theta(1));
    sf2 = exp(2*theta(2));
    s2 = exp(2*theta(3));
    
    P = ell^2*eye(length(xp), length(xp));
    
    delta_x = xp-xq;
    k = sf2*exp((-transpose(delta_x)*inv(P)*delta_x)/2) + 1/s2;
end