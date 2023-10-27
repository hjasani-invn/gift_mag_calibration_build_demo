function [ diff ] = AngleCMP( ang1, ang2 )

dA = mod( ang1 - ang2, 2*pi );

if (dA > pi)
    dA = dA - 2 * pi;
end

if (dA < -pi)
    dA = dA + 2 * pi;
end

diff = abs(dA);

end
