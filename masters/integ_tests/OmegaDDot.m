function [ Oddot ] = OmegaDDot( OmegaNext, OmegaCurr, OmegaPrev, T )
%#eml 
%OMEGADOT Function approximates OmegaDDot, the 2nd derivative of the angular
%velocity vector, composed as a skew-symmetric matrix.
% PARAMETERS
% OmegaNext - Omega(n+1), the next value; (4x4 or 3x3)
% OmegaCurr - Omega(n), the current value; (4x4 or 3x3)
% OmegaPrev - Omega(n-1), the previous value; (4x4 or 3x3)
% T - time sample (s)
% RETVAL
% Oddot - 2nd derivative estimate, (4x4 or 3x3)

% Central finite difference method
Oddot = ( OmegaNext - 2 * OmegaCurr + OmegaPrev ) / ( T^2 );

end

