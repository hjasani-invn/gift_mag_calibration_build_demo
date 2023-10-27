function [ C ] = PropagateOrientationMatrix( C, w, tau )

C = C';
C = C * expm( Kos(w) * tau );
C = C';

end

