function [ Euler_out ] = Quat2Euler( q )
%QUAT2EULER Converts quaternion parameterization into 3-2-1 Euler angles.
% PARAMETERS
% q - quaternion input, where q(1) scalar and q(2:4) vector.
% RETVAL
% Euler_out - [ phi theta psi], row vector of Euler angles
% Units - radians.

[ r c ]     = size( q );
Euler_out   = zeros( r, 4 );
phi         = zeros( r, 1 );
theta       = zeros( r, 1 );
psi         = zeros( r, 1 );

for k = 1 : r
    phi(k) = atan2( 2 * q(k,3) * q(k,4) + 2 * q(k,1) * q(k,2), ...
        2 * q(k,1)^2 + 2*q(k,4)^2 - 1 );

    theta(k) = -asin( 2 * q(k,2) * q(k,4) - 2 * q(k,1) * q(k,3) );

    psi(k) = atan2( 2 * q(k,2) * q(k,3) + 2 * q(k,1) * q(k,4), ...
        2 * q(k,1)^2 + 2 * q(k,2)^2 - 1 );
    
    % 10/22/12 edit...no negative yaw angles
    if( psi( k ) < 0 )
       psi(k) = psi(k) + 2*pi; 
    end

end
Euler_out = horzcat( phi, theta, psi );

end

