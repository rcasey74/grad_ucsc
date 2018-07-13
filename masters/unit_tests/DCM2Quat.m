function [ Quat_Out ] = DCM2Quat( R )
%DCM2QUAT Converts a 3x3 rotation matrix into its corresponding quaternion
% representation.
%   R = input Direction Cosine Matrix
%   Quat_Out = output quaternion in column vector form
%   q(0) is the scalar part, q(1-3) form the vector part.
%   Using conversion formula from Ch.7, Sect.9 (pg. 169) of 
%   J. Kuipers "Quaternions and Rotation Sequences."
    Quat_Out = single( zeros( 4, 1 ) );
    Quat_Out( 1 ) = ( 1/2 ) * sqrt( trace( R ) + 1 );
    Quat_Out( 2 ) = ( R( 2, 3 ) - R( 3, 2 ) ) / ( 4 * Quat_Out ( 1 ) ); 
    Quat_Out( 3 ) = ( R( 3, 1 ) - R( 1, 3 ) ) / ( 4 * Quat_Out ( 1 ) ); 
    Quat_Out( 4 ) = ( R( 1, 2 ) - R( 2, 1 ) ) / ( 4 * Quat_Out ( 1 ) ); 

end

