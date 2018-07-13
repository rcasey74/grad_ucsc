function [ OmBar ] = OmegaBar( Omega, Omega_dot, Omega_ddot, dt )
%#eml 
%OMEGABAR Function calculates an approximation to Omega_bar, the
%time-averaged body rates.
% PARAMETERS
% Omega - skew symmetric matrix (4x4 quaternion or 3x3 dcm)
% Omega_dot - first derivative of Omega (4x4 quaternion or 3x3 dcm)
% Omega_ddot - second derivative of Omega (4x4 quaternion or 3x3 dcm)
% dt - timestep (s)
% RETVAL
% OmBar - time-averaged matrix approximation (4x4 quaternion or 3x3 dcm)

% Formula adapted from (17-22), p. 565 of J. Wertz' 'Spacecraft Attitude
% Determination and Control'
OmBar = Omega + (1/2) * dt * Omega_dot + (1/6) * dt^2 * Omega_ddot;

end

