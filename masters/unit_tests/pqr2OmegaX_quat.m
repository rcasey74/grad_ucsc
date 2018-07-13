function [ OmegaX ] = pqr2OmegaX_quat( p, q, r )
%PQR2OMEGAX_QUAT Function creates 4x4 skew-symmetric matrix Omega for quaternion
%parameterization using angular velocities p, q, r.

% OmegaX = zeros( 4, 4 );
OmegaX = [ 0 -p -q -r; p 0 r -q; q -r 0 p; r q -p 0 ];


end

