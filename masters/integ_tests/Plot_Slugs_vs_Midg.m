% Plot_Slugs_vs_Midg.m
% 7/2/12
% Robert Casey
% Uses parms from the workspace assumed to have been populated  by the
% replay model 'ReplayFlight_DemozV2_RTC.mdl' and the 'PrepareSim.m" script


vect_len = length( roll_m.signals.values);
t_v = [ 1 : vect_len ];

sample_start = 1;  %54000; % sample #, given 9 minutes at 100hz, 60 seconds/min %450;
sample_end = vect_len; %66000;  % sample #, given 11 minutes at 100hz, 60 seconds/min  %550;
sample_range = [ sample_start : sample_end ];
sample_factor = 60 * 100; % 60 sec/min, 100 samples/sec


figure
subplot( 311 )
plot(t_v(sample_range)/sample_factor, roll_m.signals.values(sample_range), 'r-', ...
    t_v(sample_range)/sample_factor, roll_T.signals.values(sample_range), 'b-', 'linewidth',2);
legend('MIDG II Solution','SLUGS Comp Filter Soln');
title('Roll angle');
xlabel('time (min)');
ylabel('\phi (deg)');
grid on;
    
subplot(312)
plot(t_v(sample_range)/sample_factor, pitch_m.signals.values(sample_range), 'r-', ...
    t_v(sample_range)/sample_factor, pitch_T.signals.values(sample_range),'b-','linewidth',2);
title('Pitch angle');
xlabel('time (min)');
ylabel('\theta (deg)');
grid on;

subplot(313)
plot(t_v(sample_range)/sample_factor, COG_d.signals.values(sample_range),'r-', ...
    t_v(sample_range)/sample_factor, yaw_T.signals.values(sample_range),'b-','linewidth',2);
legend('GPS COG','SLUGS Comp Filter Soln');
title('COG angle');
xlabel('time (min)');
ylabel('COG (deg)');
grid on;

%% Compute Error Statistics
MSE_roll = mean(( roll_T.signals.values(sample_range) - roll_m.signals.values(sample_range)).^2 )
RMSE_roll = sqrt( MSE_roll )

MSE_pitch = mean(( pitch_T.signals.values(sample_range) - pitch_m.signals.values(sample_range)).^2 )
RMSE_pitch = sqrt( MSE_pitch )

MSE_yaw = mean(( yaw_T.signals.values(sample_range) - COG_d.signals.values(sample_range)).^2 )
RMSE_yaw = sqrt( MSE_yaw )

R = R_out.signals.values(:,:,end);
% Frobenius norm
Fnorm = norm( R, 'fro' ) 
% Orthonormality measure
Ortho = norm( eye(3) - R * R' )

% 
% error_pitch = pitch_m.signals.values(sample_range) - pitch_s.signals.values(sample_range);
% error_pitch_m = mean( error_pitch) 
% error_pitch_std = std( error_pitch )
% 
% error_yaw = COG_d.signals.values(sample_range) - yaw_s.signals.values(sample_range);
% error_yaw_m = mean( error_yaw )
% error_yaw_std = std( error_yaw )