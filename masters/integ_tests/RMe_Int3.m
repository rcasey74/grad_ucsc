function DCM_out = RMe_Int3( OmegaX, R_in, t ) 
%#eml 
%RME_INT Function performs integration using the matrix exponential
% in the direction cosine matrix parameterization.
% PARAMETERS
% OmegaX - skew symmetric, (3x3) matrix of angular velocities 
% R_in - 3x3 direction cosine matrix of last time-step's attitude
% t - sample time of system.
% RETVAL
% DCM_out - 3x3 direction cosine matrix of current time-step's attitude
% Uses matrix exponential from R. Murray, pg. 28,
% "A Mathematical Introduction to Robotic Manipulation", 1994.

% Rexp = expm( OmegaX ); 
p = OmegaX( 3, 2 );
q = OmegaX( 1, 3 );
r = OmegaX( 2, 1 );
% Om = [ p; q; r ];
wn = sqrt( p^2 + q^2 + r^2 );
Rexp = eye( 3 ) + OmegaX * sin( wn * t ) / wn + ...
    OmegaX * OmegaX * ( 1 - cos( wn * t ) ) / wn ^ 2;

DCM_out = R_in * Rexp;

end

