function [ R ] = Euler2DCM( phi, theta, psi )
%EULER2DCM Usage:  [ R ] = Euler2DCM( phi, theta, psi )
%   Take a set of Euler angles and output the corresponding DCM.
%   PARAMETERS:
%   phi - roll angle,
%   theta - pitch angle, 
%   psi - yaw angle. 
%   RETVAL
%   R - output 3x3 direction cosine matrix. (dimensionless)
%   Using the 321/ZYX Aerospace sequence of Euler angles.
%   Angular units - radians.
%
%   Return value:
%   R - the resulting 3x3 direction cosine matrix, per eq'n 2.71 of J. Craig, 
%   'Intro to Robotics and Control', 3rd Ed.

R = single( zeros( 3, 3 ) );
R = [ cos(psi)*cos(theta) ...
      cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) ...
      cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi) ; ...
      sin(psi)*cos(theta) ...
      sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) ...
      sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi); ...
      -sin(theta) ...
      cos(theta)*sin(phi) ...
      cos(theta)*cos(phi) ];
end

