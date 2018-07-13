function [ OmegaX ] = pqr2OmegaX_dcm( p, q, r )
%#eml 
%PQR2OMEGAX_QUAT Function creates 3x3 skew-symmetric matrix Omega for DCM
%parameterization using angular velocities p, q, r.

OmegaX = zeros( 3, 3 );
OmegaX = [ 0 -r q; r 0 -p; -q p 0 ];

end

