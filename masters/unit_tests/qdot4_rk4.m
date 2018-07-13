function[ dq ] = qdot4_rk4( t, y )
%QDOT3_RK4 Function calculates the change in the rotation quaternion, q_dot,
%   specifically formulated for Mathew's rk4 Matlab implementation
%   INPUTS
%   t - time sample
%   y - the previous value of the 4x1 quaternion, where y(1) scalar, y(2:4) vector
%   RETVAL
%   dq - the 1x4 row vector of changes in the rotation quaternion
%   Formulation from P. Zipfel "Modeling and Simulation of Aerospace
%   Vehicle Dynamics", Eq. 4.77

% Change from qdot3_rk4:  was using the constant previous assumption. Have
% to explicitly account for endpoints now, related to the time sample.

% For an arbitrary time sample t, we need to generate
% p, q, r
global SampleT;
global rk4NoiseTable;
global noise_mag;
global NOISE_ON;
% global p;
% global q;
% global r;

MAX_RATE = 0.25; 
SampleT_rk4 = 0.005; % 200 Hz

% [ pt qt rt ] = GetOmega( t );
% odx = floor( t / SampleT + 1);
% odx = floor( t / SampleT + 1 );
% odx = uint( t / SampleT + 1 );
% if( odx <= numel( p ) )
%     p_ = p( odx );
%     q_ = q( odx );
%     r_ = r( odx );
%     
    p_ = MAX_RATE * sin( t );
    q_ = MAX_RATE * sin( 2*t/3 );
    r_ = MAX_RATE * cos( 2*t/3 ); 
    
    % Need a time-based index into the noise table
    k = uint16(t / SampleT_rk4 + 1);
    if( NOISE_ON == 1 )
       % add noise ... pull from the right spot in the table. 
        p_ = p_ + 2 * MAX_RATE .* noise_mag .* rk4NoiseTable( k, 1 );
        q_ = q_ + 2 * MAX_RATE .* noise_mag .* rk4NoiseTable( k, 2 );
        r_ = r_ + 2 * MAX_RATE .* noise_mag .* rk4NoiseTable( k, 3 );
    end

    dq = ((1/2) * [ 0  -p_ -q_ -r_; ...
                    p_  0   r_ -q_; ...
                    q_ -r_  0   p_; ...
                    r_  q_ -p_  0 ] * y' ); % / tstep 

    dq = dq'; % return row vector
% else
%     dq = zeros( 1, 4 );  % don't care - shouldn't be here anyway
% end

end
