function [ Odot ] = OmegaDot( OmegaNext, OmegaPrev, T )
%#eml 
%OMEGADOT Function approximates OmegaDot, the first derivative of the angular
%velocity vector, composed as a skew-symmetric matrix.
% PARAMETERS
% OmegaNext - Omega(n+1), the next value; (4x4 or 3x3)
% OmegaPrev - Omega(n-1), the previous value; (4x4 or 3x3)
% T - time sample (s)
% RETVAL
% Odot - 1st derivative estimate (4x4 or 3x3)

% Central finite difference method
Odot = ( OmegaNext - OmegaPrev ) / ( 2 * T );

end

