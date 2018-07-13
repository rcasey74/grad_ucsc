% Plot_Res4.m
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
title('Results of MIDG2 data + COG vs. Complementary Filtered \muNav data with Int. Method DCM')
subplot( 311 )
plot(t_v(sample_range)/sample_factor, roll_m.signals.values(sample_range), 'r-', ...
    t_v(sample_range)/sample_factor, roll_s.signals.values(sample_range), 'b-');
legend('MIDG II Solution','DCM Soln');
title('Roll angle');
xlabel('Time (min)');
ylabel('\phi (deg)');
grid on;
    
subplot(312)
plot(t_v(sample_range)/sample_factor, pitch_m.signals.values(sample_range), 'r-', ...
    t_v(sample_range)/sample_factor, pitch_s.signals.values(sample_range),'b-');
legend('MIDG II Solution','DCM Soln');
title('Pitch angle');
xlabel('Time (min)');
ylabel('\theta (deg)');
grid on;

subplot(313)
plot(t_v(sample_range)/sample_factor, COG_d.signals.values(sample_range),'r-', ...
    t_v(sample_range)/sample_factor, yaw_s.signals.values(sample_range),'b-');
legend('MIDG II Solution','DCM Soln');
title('COG angle');
xlabel('Time (min)');
ylabel('COG (deg)');
grid on;

error_roll = roll_m.signals.values(sample_range) - roll_s.signals.values(sample_range);
error_roll_m = mean( error_roll );
error_roll_std = std( error_roll );
Roll_Error = [ error_roll_m error_roll_std ]

error_pitch = pitch_m.signals.values(sample_range) - pitch_s.signals.values(sample_range);
error_pitch_m = mean( error_pitch) ;
error_pitch_std = std( error_pitch );
Pitch_Error = [ error_pitch_m error_pitch_std ]

error_yaw = COG_d.signals.values(sample_range) - yaw_s.signals.values(sample_range);
error_yaw_m = mean( error_yaw );
error_yaw_std = std( error_yaw );
Yaw_Error = [ error_yaw_m error_yaw_std ]