function [ PhiDot ADot ] = AngAxDot2( Omega, Phi, A )
%[ PhiDot ADot ] = AngAxDot2( Omega, Phi, A )
%Function calculates the kinematics of the angle-axis
%parameterization.
%   PARAMETERS
%   Omega - 3x1 angular velocity vector
%   Phi - scalar, angle of rotation
%   A - 1x3 vector representing axis of rotation
%
%   RETVAL
%   PhiDot - scalar; change in Phi angle of rot
%   ADot - 1x3 vector, change in A axis of rot
%
%   Formulas from Eqs. 3.74, 3.75 pg.16, of C. Hall's
%   'Attitude Kinematics' lectures, Ch. 3, 'Kinematics',
%   Virginia Tech, 2003.

PhiDot = A * Omega;

% Slight abuse of notation with the following function:
% The names are different, but the function does what we
% need:  form a skew-symmetric matrix based off 3x1 vector.
Ax  = pqr2OmegaX_dcm( A(1), A(2), A(3) );

ADot = ((1/2) * ( Ax - cot( Phi / 2 ) * ( Ax * Ax ) ) * Omega)';
end

