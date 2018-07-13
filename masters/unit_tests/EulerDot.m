function [ PHI_DOT ] = EulerDot( omega, PHI_in )
%EULERDOT [ PHI_DOT ] = EulerDot( omega, PHI_in )
% Function calculates the temporal derivative 
% of the Euler angles 
% units - radians
%
% PARAMETERS:
% omega  - 3x1 vector of angular velocities, [ p q r ]'
% PHI_in - 1x3 vector of Euler angles, [ phi theta psi ]
%
% RETURN VALUE:
% PHI_DOT - 1x3 vector of Euler angle rates, [ phi_dot theta_dot psi_dot ]'

% 3-2-1 Aerospace sequence of Euler angles
% PHI = [ psi theta phi ]'
phi   = PHI_in( 1 );
theta = PHI_in( 2 );
psi   = PHI_in( 3 );

% Equation 3 from 1998 Patent 6,061,611; S. Whitmore
% "Closed-Form Integrator for the Quaternion (Euler Angle) Kinematics Equations
PHI_DOT = single(zeros( 1, 3 ));
if( theta == pi/2 )
    disp( 'theta == pi/2:  singularity!')
end

PHI_DOT = ( sec( theta ) .* ...
    [ cos( theta )   sin( phi )*sin( theta )   cos( phi )*sin( theta ) ; ...
      0              cos( phi )*cos( theta )  -sin( phi )*cos( theta ) ; ...
      0              sin( phi )                cos( phi )  ] * omega )';
  
end  
