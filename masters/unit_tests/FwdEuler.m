function [ y_next ] = FwdEuler( y_dot, dt, y_curr )
%FWDEULER Function calculates the 1st order, Forward Euler approximation 
%   method for the initial value problem, as follows:
%   y(n+1) = y(n) + dt*y'.
%   INPUTS
%   y_dot - derivative of the y vector
%   dt - time step interval
%   y_curr - current value of the y vector
%   RETVAL
%   y_next
%   
y_next = y_curr + dt * y_dot;

end

