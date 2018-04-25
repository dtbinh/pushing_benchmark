function dk = dk(i, P, x_star, x, theta)

    k = covFunc(theta, x, x_star);
    dk = -P(i,i)*(x_star(i)-x(i))*k;
end
