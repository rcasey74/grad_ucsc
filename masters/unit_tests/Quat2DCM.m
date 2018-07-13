function [ R_out ] = Quat2DCM( q )
%QUAT2DCM Function converts quaternion into direction cosine matrix
%   q = input quaternion (q(1-4))
%   R_out = output DCM
%   q(1) is the scalar part, q(2-4) form the vector part.
%   Using conversion formula from Ch.7, Sect.7 (pg. 168) of 
%   J. Kuipers "Quaternions and Rotation Sequences."

R_out = single(zeros( 3, 3 )); % memory alloc

R_out = [ 2*q(1)^2 - 1 + 2*q(2)^2      2*q(2)*q(3) + 2*q(1)*q(4)   2*q(2)*q(4) - 2*q(1)*q(3); ...
        2*q(2)*q(3) - 2*q(1)*q(4)      2*q(1)^2 - 1 + 2*q(3)^2     2*q(3)*q(4) + 2*q(1)*q(2); ...
        2*q(2)*q(4) + 2*q(1)*q(3)      2*q(3)*q(4) - 2*q(1)*q(2)   2*q(1)^2 - 1 + 2*q(4)^2 ];
end

