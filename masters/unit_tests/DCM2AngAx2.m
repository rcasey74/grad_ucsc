function [ PHI, A ] = DCM2AngAx2( R )
% DCM2ANGAX2 Function converts a direction cosine matrix into the angle-axis
% parameterization
% Inputs:  R0 - the direction cosine matrix
% Retval:  PHI - the angle, in radians
%          A   - the axis, a 3x1 column vector 
%
% Note: conversion formula from Eq. 3.13, 3.48, 3.49 of C. Hall,
%   'Attitude Kinematics' lectures, Ch. 3, 'Kinematics', Virginia Tech, 2003.

A = double( zeros( 3, 1 ) );
PHI = acos( (1/2) * ( trace( R ) - 1 ) );

% check for singularity condition
if( PHI == 0 || PHI == 2*pi )
    A = [ 0; 0; 1 ];  % arbitrary correction: use z-axis as axis of rotation
else
    % Create skew-symmetric matrix
    Ax = ( 1 / ( 2 * sin( PHI ))) * ( R' - R );
    
    % Create vector from s.s.m. above
    A = [ Ax( 3, 2 ) ; Ax( 1, 3 ) ; Ax( 2, 1 ) ];

    if( A(3) < 0 ) % Interesting edge case: z-axis negative.
        A = -A;     % Fix:  flip the whole vector so it points in +z.
    end
end %if
end %function