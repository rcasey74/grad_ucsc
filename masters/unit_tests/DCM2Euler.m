function [ Euler_out ] = DCM2Euler( R )
%DCM2EULER Function converts a Direction Cosine Matrix into its corresponding 
%   3-2-1 Euler angle parameterization.
%   PARAMETERS
%   R = input DCM, 3x3 matrix
%   RETVAL
%   Euler_out = output Euler angles in a row vector (1x3), radians
    Euler_out = single( zeros( 1, 3 ) );
    Euler_out( 1 ) = atan2( R( 3, 2 ), R( 3, 3 ));  % phi
    Euler_out( 2 ) = -asin( R( 3, 1 ));             % theta
    Euler_out( 3 ) = atan2( R( 2, 1 ), R( 1, 1 ));  % psi
    % 10/22/12 edit - no negative yaw/psi
    if( Euler_out(3) < 0 )
        Euler_out(3) = Euler_out(3) + 2 * pi;
    end
end
