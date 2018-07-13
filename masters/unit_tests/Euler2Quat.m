function [ q_out ] = Euler2Quat( phi, theta, psi )
%EULER2QUAT Function converts 3-2-1 Euler angle parameterization 
% to the rotation quaternion parm.
%
% PARAMETERS
% phi - rotation about x-axis
% theta - rotation about new y-axis
% psi - rotation about new z-axis
% Units = radians
%
% RETVAL
% q_out - the 4x1 output quaternion 

q_out = zeros( 4, 1 );

q_out( 1 ) = cos(psi/2) * cos(theta/2) * cos(phi/2) + ...
             sin(psi/2) * sin(theta/2) * sin(phi/2);
         
q_out( 2 ) = cos(psi/2) * cos(theta/2) * sin(phi/2) - ...
             sin(psi/2) * sin(theta/2) * cos(phi/2);
         
q_out( 3 ) = cos(psi/2) * sin(theta/2) * cos(phi/2) + ...
             sin(psi/2) * cos(theta/2) * sin(phi/2);
         
q_out( 4 ) = sin(psi/2) * cos(theta/2) * cos(phi/2) - ...
             cos(psi/2) * sin(theta/2) * sin(phi/2);

end

