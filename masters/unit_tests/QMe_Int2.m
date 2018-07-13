function [ Q_next ] = QMe_Int2( OmegaX, q_in, dt )
%QME_INT2 Function performs integration of the rotation quaternion
%   using the matrix exponential.
%
%   PARAMETERS
%   OmegaX - skew-symmetrix 4x4 matrix of angular velocities in body-fixed frame
%   q_in - 4x1 vector of the current attitude quaternion, q_n
%   dt - time step between samples
%   RETVAL
%   Q_next - 4x1 vector representing the value of the next quaternion 

% Formula from Eq. 17-18 of J. Wertz, 
% 'Spacecraft Attitude Determination and Control', 1978
I4 = eye( 4 );
wn = sqrt( (OmegaX(2,1))^2 + (OmegaX(3,1))^2 + (OmegaX(4,1))^2 );

% Closed-form integration
if( wn ~= 0 )
    Q_next = ( cos( ( wn * dt )/ 2 ) * I4 + ...
        (1/wn) * sin( wn * dt / 2 ) * OmegaX ) * q_in;
else
    Q_next = q_in; % (wn == 0) --> no change in q_in
end % if

end

