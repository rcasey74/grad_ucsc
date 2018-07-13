function [ R_dot ] = Rdot( OmegaX, R )
%#eml 
%RDOT Function computes Rdot, the change in the DCM from timestep to
%timestep.
% PARAMETERS
% OmegaX - the skew symmetric, (3x3) matrix of angular velocities, pqr
% R - the 3x3 DCM from the previous timestep
% RETVAL
% R_dot - the 3x3 matrix of DCM changes

R_dot = R * OmegaX;


end

