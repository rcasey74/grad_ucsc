% demoz2SlugsV5.m
% Edits by Robert T. Casey
% 10/20/12
% Version 4:  Interpolating to account for the missing data throughout. 
%             Also, analyzing interpolation results.
% Version 5:  After evaluating 2 different interp. schemes, deploy the
%             spine method for fixing the gaps in the data.

% Constants
apSampleTime = 0.01; % seconds = 100Hz rate
SampleT = 0.02;  % flight data rate = 50 Hz

%% Load original swiss-cheese data from uNav, Midg, etc.
d_data(:,2:7) = imu(:,2:7);
d_data(:,1) = t;
d_data(:,8:9) = gps_pos_lla(:,1:2)*180/pi;
d_data(:,10) = gps_pos_lla(:,3);
d_data(:,13) =  roll;
d_data(:,14) =  pitch;
d_data(:,15) =  yaw;

sog = sqrt(gps_vel_ned(:,1).^2 + gps_vel_ned(:,2).^2);
cog = atan2(gps_vel_ned(:,2), gps_vel_ned(:,1))*180/pi + 180;

d_data(:,11) = cog;
d_data(:,12) = sog;

cog = atan2(gps_vel_ned(:,2), gps_vel_ned(:,1))*180/pi;

mm = size(d_data,1);
for ii = 1:mm
    if ( cog(ii) < 0 )
        d_data(ii,17) = cog(ii) + 360;
    else
        d_data(ii,17) = cog(ii);
    end
end
d_data(:,16) =  yaw + 180;


%% Fix swiss-cheese data by interpolation
% Fix timesteps
startval = t(1);
endval = t(end);
xi = startval : SampleT : endval;  % fill the whole thing!
x = t;
y = t;
t_new = interp1( x, y, xi, 'linear' ); % should be linear timesteps

% Create new data matrix
new_size = length(t_new); % get # rows
[ nr nc ] = size(d_data); % get # cols
dat_new = zeros( new_size, nc ); 
dat_new( :, 1 ) = t_new';

% Fix all other sensor data
for cdx = 2 : 17
    dat_old = d_data( :, cdx );
    dat_new( :, cdx ) = (interp1( t, dat_old, t_new, 'spline' ))';
end % for

% Copy the new data matrix over the old one
d_data = dat_new;

%% Provide initial conditions
% MIDG2 IMU outputs - FROM DATA FILE
% Data file units are degrees
% Phi_M   = (d_data(:,13));
% Theta_M = (d_data(:,14));
% Psi_M   = (d_data(:,11));  % this is actually cog
% Euler_M = horzcat( Phi_M, Theta_M, Psi_M );
% 
% % Initial conditions, Euler angles 
% Phi_IC   = degtorad(single(Phi_M( 1 ) ))
% ePhi_IC  = (single(Phi_IC))
% Theta_IC = degtorad(single(Theta_M( 1 )))
% Psi_IC   = degtorad(single(d_data(1,11)))
% QUAT_0   = single(Euler2Quat( (Phi_IC), (Theta_IC), (Psi_IC) )) % quaternion
% R0       = single(Euler2DCM( ePhi_IC, Theta_IC, Psi_IC )) % DCM
% [aPhi_IC aAxis_IC ] = DCM2AngAx2( R0 )  % Angle Axis

% ye olde hard code ... for now
PHI_IC = 0.4400000;
% PHI_IC = degtorad( PHI_IC )

THETA_IC = -1.0900000;
% THETA_IC = degtorad( THETA_IC )

PSI_IC =  57.7712440;
% PSI_IC = degtorad( PSI_IC )

R0 = single(Euler2DCMdeg( PHI_IC, THETA_IC, PSI_IC ))
QUAT_0 = DCM2Quat( R0 )
% R0 = [ -0.9397 0.3420 0; -0.3420 -0.9397 0 ; 0 0 1.0000 ]; 
