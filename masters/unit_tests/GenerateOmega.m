function [ p_ q_ r_ ] = GenerateOmega( t, test_mode )
%GETOMEGA Function generates appropriate angular velocity p, q, r values
%according to the input timestep, t.
% PARAMETERS
% t - time sample array, in seconds
% RETURN VALUE
% p - body-fixed x axis ang. vel
% q - body-fixed y axis ang. vel
% r - body-fixed z axis ang. vel
% Units - radians

global SampleT;
global NoiseTable;
global noise_mag;
global NOISE_ON;

% Test configurations and modes
MAX_RATE = 0.25; % rad/sec
MAX_T    = 10;

% Test modes, per input waveform type
TEST_STEP            = 1;
TEST_DOUBLET         = 2;
TEST_RAMP            = 3;
TEST_SINUSOID        = 4;
TEST_DAMPED_SINUSOID = 5;
test_mode            = TEST_SINUSOID;

switch test_mode
    case TEST_STEP
        if( t < MAX_T )
           p_ = 0;
           q_ = 0;
           r_ = 0;
        else
            p_ = MAX_RATE;
            q_ = MAX_RATE;
            r_ = MAX_RATE;
        end

    case TEST_DOUBLET
        if( t(end) == 10 )
             % Range 1:  0 - 2s, constant
               t_start = 1;
               t_end = find( t == 2 );
               p_(t_start : t_end ) = 0;
               q_(t_start : t_end ) = 0;
               r_(t_start : t_end ) = 0;
             % Range 2:  2 - 2.5s, sinusoidal increase
               t_start = t_end;
               t_end = find( t == 2.5 );
               p_(t_start : t_end ) = MAX_RATE * sin( pi * t(t_start : t_end ) );
               q_(t_start : t_end ) = MAX_RATE * sin( pi * t(t_start : t_end ) );
               r_(t_start : t_end ) = MAX_RATE * sin( pi * t(t_start : t_end ) );
             % Range 3:  2.5 - 4.5s, constant               
               t_start = t_end;
               t_end = find( t == 4.5 );
               p_(t_start : t_end ) = MAX_RATE;
               q_(t_start : t_end ) = MAX_RATE;
               r_(t_start : t_end ) = MAX_RATE;
             % Range 4:  4.5 - 5.5s, sinusoidal decrease
               t_start = t_end;
               t_end = find( t == 5.5 );             
               p_(t_start : t_end ) = MAX_RATE * sin( pi * t(t_start : t_end ) );
               q_(t_start : t_end ) = MAX_RATE * sin( pi * t(t_start : t_end ) );
               r_(t_start : t_end ) = MAX_RATE * sin( pi * t(t_start : t_end ) );
             % Range 5:  5.5 - 7.5s, constant
               t_start = t_end;
               t_end = find( t == 7.5 );             
               p_(t_start : t_end ) = -MAX_RATE;
               q_(t_start : t_end ) = -MAX_RATE;
               r_(t_start : t_end ) = -MAX_RATE;
             % Range 6:  7.5 - 8.0s, sinusoidal increase
               t_start = t_end;
               t_end = find( t == 8.0 );             
               p_(t_start : t_end ) = MAX_RATE * sin( pi * t(t_start : t_end ) );
               q_(t_start : t_end ) = MAX_RATE * sin( pi * t(t_start : t_end ) );
               r_(t_start : t_end ) = MAX_RATE * sin( pi * t(t_start : t_end ) );
             % Range 7:  8.0 - 10s, constant
               t_start = t_end;
               t_end = find( t == 10 );             
               p_(t_start : t_end ) = 0;
               q_(t_start : t_end ) = 0;
               r_(t_start : t_end ) = 0;              
%             end % time range
        else
           disp( 'Need to generalize doublet function pqr inputs for t_max ~= 10s!' ) 
        end % max_t == 10
        
    case TEST_RAMP
        p_ = 1;
        q_ = 1;
        r_ = 1;
        
    case TEST_SINUSOID
        p_ = MAX_RATE * sin( t );
        q_ = MAX_RATE * sin( 2*t/3 );
        r_ = MAX_RATE * cos( 2*t/3 );
        
    case TEST_DAMPED_SINUSOID        
        p_ = exp( -t ) * MAX_RATE * sin( t );
        q_ = exp( -(1/2)*t ) * MAX_RATE * sin( 2*t/3 );
        r_ = exp( -(1/4)*t ) * MAX_RATE * cos( 2*t/3 );
        
end % switch

end




