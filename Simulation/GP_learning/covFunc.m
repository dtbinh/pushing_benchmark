function [k, dk] = covFunc(theta, xp, xq, scale_factor)
    ell1 = exp(theta(1));
    ell2 = exp(theta(2));
    ell3 = exp(theta(3));
    sf2 = exp(2*theta(4));
    s2 = exp(2*theta(5));
    
    P = diag([ell1^2,ell2^2,ell3^2]);
    
    delta_x = xp-repmat(xq, 1, size(xp,2));
%     k = sf2*exp((-dot(delta_x, inv(P)*delta_x))/2) + 1/s2;
    k =  sf2*exp((-transpose(delta_x)*inv(P)*delta_x)/2) + 1/s2;
    dk = (sf2*exp((-transpose(delta_x)*inv(P)*delta_x)/2)*inv(P)*(2*xp-2*repmat(xq, 1, size(xp,2)))./scale_factor)/2; 
%     dk = (sf2*exp((-dot(delta_x, inv(P)*delta_x))/2)*inv(P)*(2*xp-2*repmat(xq, 1, size(xp,2))))/2; %sf2*exp(-(xp - xq)^2/2)*inv(P)*(xp - xq);
end