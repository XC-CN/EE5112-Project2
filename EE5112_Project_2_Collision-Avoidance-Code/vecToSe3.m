function se3mat = vecToSe3(S)
%VECTOSE3 Convert a 6x1 twist vector into an se(3) matrix.
omega = S(1:3);
v = S(4:6);
se3mat = [vecToSkew(omega), v; 0 0 0 0];
end

function skewMat = vecToSkew(omega)
skewMat = [    0      -omega(3)  omega(2);
           omega(3)       0     -omega(1);
          -omega(2)   omega(1)       0   ];
end
