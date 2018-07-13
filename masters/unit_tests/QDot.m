function [ q_dot ] = QDot( p, q, r, q_in )
%QDOT Function calculates the change in the rotation quaternion, q_dot
%   INPUTS
%   p, q, r - the body-fixed angular velocities (each scalar)
%   q_in - the initial 4x1 quaternion, where q_in(1) scalar, q_in(2:4) vector
%   RETVAL
%   q_dot - the 4x1 vector of changes in the rotation quaternion
%   Formulation from P. Zipfel "Modeling and Simulation of Aerospace
%   Vehicle Dynamics", Eq. 4.77

q_dot = zeros( 4, 1 );
q_dot = (1/2) * [ 0 -p -q -r; p 0 r -q; q -r 0 p; r q -p 0 ] * q_in;

end

