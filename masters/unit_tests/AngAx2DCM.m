function [ R ] = AngAx2DCM( phi, a1, a2, a3 )
%ANGLEAXIS The angle-axis representation of rotations.
%   PARAMETERS:
%   a1, a2, a3 - the x, y, and z components of the axis of rotation. Aka v.
%   phi - the numeric value of the angle of rotation, i.e. how much we wish 
%   to rotate around v, in radians.
%   RETVAL
%   R - the 3x3 direction cosine matrix
%
%   Implementation taken from equation 7.16, Section 7.5, Angle and Axis of
%   Rotation, from J. Kuipers 'Quaternions and Rotation Sequences',
%   Princeton Univ. Press, 1999. 

R = [ a1^2+(a2^2+a3^2)*cos(phi)      a1*a2*(1-cos(phi))-a3*sin(phi) a1*a3*(1-cos(phi))+a2*sin(phi); ...
      a1*a2*(1-cos(phi))+a3*sin(phi) a2^2+(a3^2+a1^2)*cos(phi)      a2*a3*(1-cos(phi))-a1*sin(phi); ...
      a3*a1*(1-cos(phi))-a2*sin(phi) a2*a3*(1-cos(phi))+a1*sin(phi) a3^2+(a1^2+a2^2)*cos(phi) ];
  
end

