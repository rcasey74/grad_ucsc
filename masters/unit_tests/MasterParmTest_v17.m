% MasterParmTest_v17.m
% Robert T. Casey  
% 12/12/12
% Script to use Mathews' RK4 Matlab routine to generate
% truth based off analytic functions of pqr.  These pqr inputs are then
% fed to each of the parameterization and integration algorithms.
% Code based on My_rk4b.m
% Version 2 changes:  - DCM parameterization & int algorithms added.
%                     - Added explicit timestep computations to int.
%                     routines.  Results were too large otherwise.  Plus it
%                     matches what the theory prescribes.
% Version 3 changes:  - Euler angle parm. & int. algs. added.
% Version 4 changes:  - Quaternion parm. & int algs.
% Version 5 changes:  - Angle-axis parm & int algs.
% Version 7 changes:  - Expanding previous tests to be full noise tests,
% with complete statistics created.  Also, stats and figures will be saved
% to file.
% Version 9 changes:  - Adding plots for the difference signal between the
% clean angular outputs and the noisy angular outputs.
% Version 10 changes: - Concatenating difference signals from each test,
% then plotting a histogram for each method.
% Version 11 changes: - Putzing around with various histogram distribution
% fit functions to identify the distribution associated with squared
% difference signal data.
% Version 12 changes:  - Redoing the clean-signal error statistics  to evaluate the root
% mean squared error, not just the mean error of before, which was
% uber-misleading.
% Version 13 changes: - Redoing Version 11 changes - preparing data for
% distribution fitting, using the revised error statistics from version 12.
% Need the following output signals:  (AlgoNoise - RK4Clean)^2,
% (RK4Noise-RK4Clean)^2.
% Version 14 changes: - Per committee feedback, have to change the 
% noise signal added to be a percentage of the peak-to-peak magnitude.
% Version 15 changes: - Redoing accuracy test plots of error (residual)
% signal so they are in units of degrees, not radians.
% Version 16 changes:  - Redoing RK4 formulation using angular velocities,
% etc. using analytic function, up sample, not just constant previous.
% Version 17 changes:  - Extending v16 to handle noise tests, with
% Algo_Noise - Algo_Clean being the signal of interest here.
% ------------------------------------------------------------------------
global SampleT;
global rk4NoiseTable;
global noise_mag;
global NOISE_ON;
% global p;
% global q;
% global r;

% Set/clear flags to enable/disable testing of individual parms.
% HIGHLY RECOMMENDED:  TEST ONLY ONE (1) PARAMETERIZATION AT A TIME!!!
EULER_TEST = 0;
QUAT_TEST  = 0;
DCM_TEST   = 1;
AA_TEST    = 0;

NUM_NOISE_RUNS = 10;  % count
RUN_CLEAN = 1;  % id

MAX_RATE = 0.25;  % angular velocity peak - rad
noise_mag = 1; % default
retval = []; % throwaway

% Test modes, per input waveform type
TEST_STEP            = 1;
TEST_DOUBLET         = 2;
TEST_RAMP            = 3;
TEST_SINUSOID        = 4;
TEST_DAMPED_SINUSOID = 5;
test_mode            = TEST_SINUSOID;

dir = 'FinDat';

% Test noise settings
% Noise settings
NOISE_1              = 1;
NOISE_MAG_1          = 0.1;
NOISE_2              = 2;
NOISE_MAG_2          = 0.01;
NOISE_3              = 3;
NOISE_MAG_3          = 0.001;

% Choose settings
NOISE_ON    =       1; 
noise_level = NOISE_1;

load('dat\MasterNoiseTable2.mat');  % for RK4 - 200Hz sampled
load('dat\AlgoNoiseTable.mat');     % for algorithm testing - 100 Hz sampled
switch noise_level
    case NOISE_1
       noise_mag = NOISE_MAG_1; 
    case NOISE_2
       noise_mag = NOISE_MAG_2;
    case NOISE_3
       noise_mag = NOISE_MAG_3;           
end % switch

switch test_mode 
    case TEST_STEP
        mode_str = 'STEP';
    case TEST_DOUBLET
        mode_str = 'DOUBLET'; 
    case TEST_RAMP
        mode_str = 'RAMP';    
    case TEST_SINUSOID
        mode_str = 'SINUSOID';
    case TEST_DAMPED_SINUSOID
        mode_str = 'DAMPED SINUSOID';                
end

% Column indices
C_phi   = 1;
C_theta = 2;
C_psi   = 3;

% INITIALIZE THE DATA ENVIRONMENT
% --------------------------------
% Data environment will depend on the specific tests
% being executed (step vs. doublet vs. ramp vs. sinusoid).

% Initial condition for Euler angles - 0 roll, 0 pitch, and pi/6 yaw
% Units - radians
% This will require some adjustment when handling uNav data.
ePhi_IC = 0;
Theta_IC = 0;
Psi_IC = pi/6;

R0 = Euler2DCM( ePhi_IC, Theta_IC, Psi_IC ); % DCM
QUAT_0 = Euler2Quat( ePhi_IC, Theta_IC, Psi_IC ); % Quaternion
[ aPhi_IC aAxis_IC ] = DCM2AngAx2( R0 );  % Angle Axis

SampleT     = 0.01;  
Sample100   = 0.01;
SampleFact  = SampleT / Sample100;
t_max       = 10; % seconds
t           = 0 : SampleT : t_max;
NumSamples  = t_max / SampleT;
t_idx       = 1 : NumSamples + 1;
base_m      = NumSamples;

NoiseTable    = zeros( NumSamples + 1, 3 ); % local window of noise for algos
rk4NoiseTable = zeros( NumSamples * 2 + 1, 3 ); % local window of noise for rk4
diffE_q = [];
mDiff_rk4 = [];
mT = []; % master time table
rk4_sqerr = zeros( NumSamples + 1, 3 );

if( DCM_TEST == 1 )
        mRMSE_rFec  = [];
        mRMSE_rFet  = [];
        mRMSE_rMecp = [];
        mRMSE_rMecc = [];
        mRMSE_rMet  = [];
        mRMSE_rMeb  = [];
        
        mDiff_rFec  = [];
        mDiff_rFet  = [];
        mDiff_rMecp = [];
        mDiff_rMecc = [];
        mDiff_rMet  = [];
        mDiff_rMeb  = [];         

end

if( EULER_TEST == 1 )
        mRMSE_eFec  = [];
        mRMSE_eFet  = [];
        
        mDiff_eFec  = [];
        mDiff_eFet  = [];
end

if( QUAT_TEST == 1 )
        mRMSE_qFec  = [];
        mRMSE_qFet  = [];
        mRMSE_qMecp = [];
        mRMSE_qMecc = [];
        mRMSE_qMet  = [];
        mRMSE_qMeb  = [];   
        
        mDiff_qFec  = [];
        mDiff_qFet  = [];
        mDiff_qMecp = [];
        mDiff_qMecc = [];
        mDiff_qMet  = [];
        mDiff_qMeb  = [];           
end

if( AA_TEST == 1 )
        mRMSE_aFec  = [];
        mRMSE_aFet  = [];   
        
        mDiff_aFec  = [];
        mDiff_aFet  = [];  
        
end

% end init for all tests
% ========================================================================
% begin looping tests

% Create statistics-tracking structures
% p = NUM_NOISE_RUNS;
% stats = struct( 'Max', zeros(p,1), 'Min', zeros(p,1), 'Med', zeros(p,1), ...
%     'Mu', ones(p,1), 'Sigma', zeros( p,1), 'RMSE', zeros(p,1) );

for idx = 1 : NUM_NOISE_RUNS + 1 % +1 for clean test
    str_run_mode = '';
    if( idx == RUN_CLEAN )
        str_run_mode = '-Clean';
    else
        str_run_mode = '-Noise#';
        num_noise_run = idx - 1;
        str_run_mode = strcat( str_run_mode, num2str( num_noise_run ) );
        
        mT = vertcat( mT, t' + t_max * ( num_noise_run - 1 ));
%         if( num_noise_run ~= NUM_NOISE_RUNS )
%           mT(end) = [];
%         end

    rk4NoiseTable  = MNT( 1 + ( num_noise_run - 1 ) * 2 * NumSamples * SampleFact : ...
                          1 +         num_noise_run * 2 * NumSamples * SampleFact, : );
   end
    
    disp( strcat( '--- Test ', str_run_mode, ' ---' ))
    
      
    % Use RK4 to generate Euler "truth"
    f = @qdot4_rk4; % Handle to derivative function
    a = 0;          % Start boundary condition
    b = t_max;      % End boundary condition
    ya = QUAT_0;    % Initial condition for attitude
    m = NumSamples; % How many slices to take
    [ qT, qY ] = rk4_q( f, a, b, ya, m );  % this function accesses global pqr

    % plot( qT, qY ), title( 'Raw Quaternion Output of RK4' )
    % legend( 'q1', 'q2', 'q3', 'q4' )
    % ylabel( 'Quat value' )
    % xlabel( 'Time (s)')

    % Convert to Euler angles
    [ E_q ] = Quat2Euler( qY );
    Phi_T   = E_q( :, C_phi   );
    Theta_T = E_q( :, C_theta );
    Psi_T   = E_q( :, C_psi  );
    
    if( idx == RUN_CLEAN )
        cE_q     = E_q;
        cPhi_T   = Phi_T;
        cTheta_T = Theta_T;
        cPsi_T   = Psi_T;
    end % run clean

    h_angvel = figure;
    plot( t', radtodeg(p), 'b', t', radtodeg(q), 'r', t', radtodeg(r), 'g' );
    legend( 'p', 'q', 'r' )
    title( strcat( 'Angular velocities, ', str_run_mode ) )
    xlabel( 'Time(s)' )
    ylabel( '\Omega (deg/s)' )
    savefile = strcat( 'dat\', dir, '\AngVel', str_run_mode, '.fig' );
    saveas( h_angvel, savefile );

    % "True" Euler (up to 1e-9) based off RK4
    figure
    plot( t', radtodeg(E_q), 'linewidth',2);  
    title( strcat( 'Euler True per RK4, ', str_run_mode ) )
    legend( '\phi', '\theta', '\psi' );
    xlabel( 'Time (s)')
    ylabel( '\Phi (deg)' )
    
    % Calculate stats for noisy runs
    if( idx ~= RUN_CLEAN )
        % calculate RK4 squared residuals for each run
        for sdx = 1 : NumSamples
            for edx = 1 : 3  % euler angle
                rk4_sqerr( sdx, edx )  = ( E_q( sdx, edx )  - cE_q( sdx, edx ))^2;
            end
        end
        
        RK4_Err = E_q - cE_q;
      
        % append to the master array
        diffE_q = vertcat( diffE_q, rk4_sqerr );
        
        % residuals master array
        mDiff_rk4 = vertcat( mDiff_rk4, radtodeg(RK4_Err) );
    end
    
    % ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % ====================================================================
    %                  R U N    P A R M    T E S T S  
    % ====================================================================
    % ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    % Generate sample-based pqr for the different algorithms
    % Note:  RK4 uses time-based pqr, whose every other values should match
    % up with those used by the algorithms    
    p = zeros( NumSamples + 1, 1 );
    q = zeros( NumSamples + 1, 1 );
    r = zeros( NumSamples + 1, 1 );
    [ p q r ] = GenerateOmega( t, test_mode ); % radians 
    
    % Select the appropriate window of the noise table
    if( idx ~= RUN_CLEAN )
        % Ex. 1-1001, 1001-2001, etc.
        NoiseTable = AlgoNoiseTable( 1 + ( num_noise_run - 1 ) * NumSamples * SampleFact : ...
            1 + num_noise_run * NumSamples * SampleFact, : );

        % Add noise appropriately
        for k = 1 : NumSamples + 1 
            p(k) = p(k) + 2 * MAX_RATE .* noise_mag .* NoiseTable( 1 + (k - 1) * SampleFact, 1 );
            q(k) = q(k) + 2 * MAX_RATE .* noise_mag .* NoiseTable( 1 + (k - 1) * SampleFact, 2 );
            r(k) = r(k) + 2 * MAX_RATE .* noise_mag .* NoiseTable( 1 + (k - 1) * SampleFact, 3 );
        end % end pqr noise addition
    end % if clean run    
    
    % Check results via plots
    h_angvel = figure;
    plot( t', radtodeg(p), 'b', t', radtodeg(q), 'r', t', radtodeg(r), 'g' );
    legend( 'p', 'q', 'r' )
    title( strcat( 'Angular velocities, ', str_run_mode ) )
    xlabel( 'Time(s)' )
    ylabel( '\Omega (deg/s)' )
    savefile = strcat( 'dat\', dir, '\AngVel', str_run_mode, '.fig' );
    saveas( h_angvel, savefile );    
    
    %% ======================================
    %          D C M 
    % ======================================

    if( DCM_TEST == 1 )
        disp( 'DCM IN PROGRESS --------------  ' )
        % Preallocate memory
        DCM_FE_C  = zeros( 3, 3, NumSamples + 1 );
        DCM_FE_T  = zeros( 3, 3, NumSamples + 1 );
        DCM_ME_cp = zeros( 3, 3, NumSamples + 1 );
        DCM_ME_cc = zeros( 3, 3, NumSamples + 1 );
        DCM_ME_t  = zeros( 3, 3, NumSamples + 1 );
        DCM_ME_b  = zeros( 3, 3, NumSamples + 1 );

        E_r_fe_c  = zeros( NumSamples + 1, 3 );
        E_r_fe_t  = zeros( NumSamples + 1, 3 );
        E_r_me_cp = zeros( NumSamples + 1, 3 );
        E_r_me_cc = zeros( NumSamples + 1, 3 );
        E_r_me_t  = zeros( NumSamples + 1, 3 );
        E_r_me_b  = zeros( NumSamples + 1, 3 );

        RMSE_rFec  = zeros( NumSamples + 1, 3 );
        RMSE_rFet  = zeros( NumSamples + 1, 3 );
        RMSE_rMecp = zeros( NumSamples + 1, 3 );
        RMSE_rMecc = zeros( NumSamples + 1, 3 );
        RMSE_rMet  = zeros( NumSamples + 1, 3 );
        RMSE_rMeb  = zeros( NumSamples + 1, 3 );
        
       
        % -------------------------------------
        %     F O R W A R D    E U L E R
        % -------------------------------------
        disp( 'Forward Euler ' )
        DCM_FE_C( :, :, 1 ) = R0; % IC
        for k = 1 : NumSamples 
            % CONSTANT OMEGA
            dOmegaX_c = pqr2OmegaX_dcm( p(k), q(k), r(k) );
            dcm_dot_c = Rdot( dOmegaX_c, DCM_FE_C( :, :, k )); 
            DCM_FE_C( :, :, k + 1 ) = FwdEuler( dcm_dot_c, SampleT, DCM_FE_C( :, :, k ) );
        end

        DCM_FE_T( :, :, 1 ) = R0; % IC
        for k = 1 : NumSamples 
            % TRAPEZOIDAL AVERAGE OF OMEGA
            p_t = (1/2)*( p(k+1) + p(k) );
            q_t = (1/2)*( q(k+1) + q(k) );
            r_t = (1/2)*( r(k+1) + r(k) );

            dOmegaX_t = pqr2OmegaX_dcm( p_t, q_t, r_t );
            dcm_dot_t = Rdot( dOmegaX_t, DCM_FE_T(:, :, k )); 
            DCM_FE_T( :, :, k + 1 ) = FwdEuler( dcm_dot_t, SampleT, DCM_FE_T( :, :, k ) );
        end
        % -------------------------------------
        % M A T R I X   E X P O N E N T I A L
        % -------------------------------------
        disp( 'Matrix Exponential  ' )
        DCM_ME_cp( :, :, 1 ) = R0;   % IC1
        for k = 1 : NumSamples 
        %   USE CONSTANT Wn (previous)
            p_cp = p(k);
            q_cp = q(k);
            r_cp = r(k);
            dOmegaX_cp = pqr2OmegaX_dcm( p_cp, q_cp, r_cp );
            DCM_ME_cp( :, :, k + 1 ) = RMe_Int3( dOmegaX_cp, DCM_ME_cp(:, :, k ), SampleT ); 
        end

        DCM_ME_cc( :, :, 1 ) = R0;   % IC1
        for k = 1 : NumSamples 
        %   USE CONSTANT Wn+1 (current)
            p_cc = p(k+1);
            q_cc = q(k+1);
            r_cc = r(k+1);
            dOmegaX_cc = pqr2OmegaX_dcm( p_cc, q_cc, r_cc );
            DCM_ME_cc( :, :, k + 1 ) = RMe_Int3( dOmegaX_cc, DCM_ME_cc(:, :, k ), SampleT ); 
        end

        DCM_ME_t( :, :, 1 ) = R0;   % IC1
        for k = 1 : NumSamples 
        %   USE TRAPEZOIDAL AVG:  (1/2)*(Wn+1 + Wn)
            p_t = (1/2)*( p(k+1) + p(k) );
            q_t = (1/2)*( q(k+1) + q(k) );
            r_t = (1/2)*( r(k+1) + r(k) );
            dOmegaX_met = pqr2OmegaX_dcm( p_t, q_t, r_t );
            DCM_ME_t( :, :, k + 1 ) = RMe_Int3( dOmegaX_met, DCM_ME_t( :, :, k ), SampleT ); 
        end

        % For the Omega_bar approach, we need two initial conditions due
        % to the central finite differencing method. Otherwise, QME(2,:) will
        % be zero.  Use the standard setup of instantaneous omega(tn).  
        DCM_ME_b( :, :, 1 ) = R0;   % IC1
        dOmegaX = pqr2OmegaX_dcm( p( 2 ), q( 2 ), r( 2 ) ); 
        DCM_ME_b( :, :, 2 ) = RMe_Int3( dOmegaX, DCM_ME_b( :, :, 1 ), SampleT );  % IC2
        for k = 2 : NumSamples
        %    if( k < NumSamples )
                OxNext = pqr2OmegaX_dcm( p( k + 1 ), q( k + 1 ), r( k + 1 ));
                OxCurr = pqr2OmegaX_dcm( p( k     ), q( k     ), r( k     ));
                OxPrev = pqr2OmegaX_dcm( p( k - 1 ), q( k - 1 ), r( k - 1 ));

                Odot  = OmegaDot( OxNext, OxPrev, SampleT );
                Oddot = OmegaDDot( OxNext, OxCurr, OxPrev, SampleT );
                OmBar = OmegaBar( OxCurr, Odot, Oddot, SampleT );

                DCM_ME_b( :, :, k + 1 ) = RMe_Int3( OmBar, DCM_ME_b( :, :, k ), SampleT );    
        %    end
        end

        % Convert results from DCM to Euler angles
        for k = 1 : NumSamples + 1
            % Retval is row vector
            E_r_fe_c( k, : )  = DCM2Euler( DCM_FE_C( :, :, k )); % each row = 1 timestep of [ phi theta psi ]
            E_r_fe_t( k, : )  = DCM2Euler( DCM_FE_T( :, :, k ));
            E_r_me_cp( k, : ) = DCM2Euler( DCM_ME_cp( :, :, k ));
            E_r_me_cc( k, : ) = DCM2Euler( DCM_ME_cc( :, :, k ));
            E_r_me_t( k, : )  = DCM2Euler( DCM_ME_t( :, :, k ));
            E_r_me_b( k, : )  = DCM2Euler( DCM_ME_b( :, :, k ));
        end % convert DCM -> Euler

        if( idx == RUN_CLEAN )
            cE_r_fe_c( :, : ) = E_r_fe_c( :, : );
            cE_r_fe_t( :, : ) = E_r_fe_t( :, : );
            cE_r_me_cp( :, : ) = E_r_me_cp( :, : );
            cE_r_me_cc( :, : ) = E_r_me_cc( :, : );
            cE_r_me_t( :, : ) = E_r_me_t( :, : );
            cE_r_me_b( :, : ) = E_r_me_b( :, : );
        else % Noise testing!
            % CALCULATE RMSE STATS 
            % Calculate residuals - Error from difference signal of
            % Algo_Noise - Algo_Clean
            Rerr_Fe_c   = E_r_fe_c  - cE_r_fe_c;  
            Rerr_Fe_t   = E_r_fe_t  - cE_r_fe_t;
            Rerr_Me_cp  = E_r_me_cp - cE_r_me_cp;  
            Rerr_Me_cc  = E_r_me_cc - cE_r_me_cc;  
            Rerr_Me_t   = E_r_me_t  - cE_r_me_t;  
            Rerr_Me_b   = E_r_me_b  - cE_r_me_b;  

            % Square residuals
            for rdx = 1 : NumSamples + 1
                for edx = 1 : 3
                    RMSE_rFec( rdx, edx )  = ( (Rerr_Fe_c(  rdx, edx ))^2 ); 
                    RMSE_rFet( rdx, edx )  = ( (Rerr_Fe_t(  rdx, edx ))^2 ); 
                    RMSE_rMecp( rdx, edx ) = ( (Rerr_Me_cp( rdx, edx ))^2 ); 
                    RMSE_rMecc( rdx, edx ) = ( (Rerr_Me_cc( rdx, edx ))^2 ); 
                    RMSE_rMet( rdx, edx )  = ( (Rerr_Me_t(  rdx, edx ))^2 ); 
                    RMSE_rMeb( rdx, edx )  = ( (Rerr_Me_b(  rdx, edx ))^2 ); 
                end % for edx
            end     % for rdx

           % Append squared residuals to master array
            mRMSE_rFec  = vertcat( mRMSE_rFec, RMSE_rFec) ;
            mRMSE_rFet  = vertcat( mRMSE_rFet, RMSE_rFet) ;
            mRMSE_rMecp = vertcat( mRMSE_rMecp, RMSE_rMecp) ;
            mRMSE_rMecc = vertcat( mRMSE_rMecc, RMSE_rMecc) ;
            mRMSE_rMet  = vertcat( mRMSE_rMet, RMSE_rMet) ;
            mRMSE_rMeb  = vertcat( mRMSE_rMeb, RMSE_rMeb) ;

            % Append residuals to master array
            mDiff_rFec  = vertcat( mDiff_rFec, radtodeg(Rerr_Fe_c ));
            mDiff_rFet  = vertcat( mDiff_rFet, radtodeg(Rerr_Fe_t));
            mDiff_rMecp = vertcat( mDiff_rMecp, radtodeg(Rerr_Me_cp )); 
            mDiff_rMecc = vertcat( mDiff_rMecc, radtodeg(Rerr_Me_cc ));
            mDiff_rMet  = vertcat( mDiff_rMet, radtodeg(Rerr_Me_t));
            mDiff_rMeb  = vertcat( mDiff_rMeb, radtodeg(Rerr_Me_b));

        end % if run_clean 
    end % dcm_test

    %% ======================================
    %          E U L E R  A N G L E S
    % ======================================
    if( EULER_TEST == 1 )
        disp( 'EULER ANGLES IN PROGRESS: ----------------  ' )
        % PREALLOCATE MEMORY
        E_e_fe_c  = zeros( NumSamples + 1, 3 );
        E_e_fe_t  = zeros( NumSamples + 1, 3 );
        
        RMSE_eFec  = zeros( NumSamples + 1, 3 );
        RMSE_eFet  = zeros( NumSamples + 1, 3 );
        

        % -------------------------------------
        %     F O R W A R D    E U L E R
        % -------------------------------------
        E_e_fe_c( 1, C_phi )   = ePhi_IC; 
        E_e_fe_c( 1, C_theta ) = Theta_IC; 
        E_e_fe_c( 1, C_psi )   = Psi_IC; 
        for k = 1 : NumSamples 
            % CONSTANT OMEGA
            eOmegaX_c = [ p(k); q(k); r(k) ];
            e_dot_c = EulerDot( eOmegaX_c, E_e_fe_c( k, : )); 
            E_e_fe_c( k + 1, : ) = FwdEuler( e_dot_c, SampleT, E_e_fe_c( k, : ));
        end

        E_e_fe_t( 1, C_phi )   = ePhi_IC; 
        E_e_fe_t( 1, C_theta ) = Theta_IC; 
        E_e_fe_t( 1, C_psi )   = Psi_IC; 
        for k = 1 : NumSamples 
            % TRAPEZOIDAL AVERAGE OF OMEGA
            p_t = (1/2)*( p(k+1) + p(k) );
            q_t = (1/2)*( q(k+1) + q(k) );
            r_t = (1/2)*( r(k+1) + r(k) );

            eOmegaX_t = [ p(k); q(k); r(k) ];
            e_dot_t = EulerDot( eOmegaX_t, E_e_fe_t( k, : )); 
            E_e_fe_t( k + 1, : ) = FwdEuler( e_dot_t, SampleT, E_e_fe_t( k, : ));
        end
        
        if( idx == RUN_CLEAN )
            cE_e_fe_c( :, : )  = E_e_fe_c( :, : );
            cE_e_fe_t( :, : )  = E_e_fe_t( :, : );

        else % Noise testing!        
            % CALCULATE RMSE STATS 
            % Calculate residuals - Error from 
            % Algo_Noise - Algo_Clean        
            Eerr_Fe_c   = E_e_fe_c  - cE_e_fe_c;  
            Eerr_Fe_t   = E_e_fe_t  - cE_e_fe_t;

            % Square residuals
            for rdx = 1 : NumSamples + 1
                for edx = 1 : 3
                    RMSE_eFec( rdx, edx )  = ( (Eerr_Fe_c(  rdx, edx ))^2 ); 
                    RMSE_eFet( rdx, edx )  = ( (Eerr_Fe_t(  rdx, edx ))^2 ); 
                end
            end

            % Append squared residuals to master array
            mRMSE_eFec  = vertcat( mRMSE_eFec, RMSE_eFec) ;
            mRMSE_eFet  = vertcat( mRMSE_eFet, RMSE_eFet) ;
            
            % Append residuals to master array     
            mDiff_eFec  = vertcat( mDiff_eFec, radtodeg( Eerr_Fe_c ));
            mDiff_eFet  = vertcat( mDiff_eFet, radtodeg(Eerr_Fe_t ));
        
        end % if run_clean 
    end % euler_test

    %% ======================================
    %          Q U A T E R N I O N S  
    % ======================================
    if( QUAT_TEST == 1 )
        disp( 'QUATERNIONS IN PROGRESS:  -------------- ' )
        % PREALLOCATE MEMORY
        quat_dot_c = zeros( 4, 1 );
        quat_dot_t = zeros( 4, 1 );

        QUAT_FE_C  = zeros( NumSamples + 1, 4 );
        QUAT_FE_T  = zeros( NumSamples + 1, 4 );
        E_q_fe_c   = zeros( NumSamples + 1, 3 );
        E_q_fe_t   = zeros( NumSamples + 1, 3 );

        QUAT_ME_cp = zeros( NumSamples + 1, 4 );
        QUAT_ME_cc = zeros( NumSamples + 1, 4 );
        QUAT_ME_t  = zeros( NumSamples + 1, 4 );
        QUAT_ME_b  = zeros( NumSamples + 1, 4 );

        E_q_me_cp  = zeros( NumSamples + 1, 3 );
        E_q_me_cc  = zeros( NumSamples + 1, 3 );
        E_q_me_t   = zeros( NumSamples + 1, 3 );
        E_q_me_b   = zeros( NumSamples + 1, 3 );
        
%         RMSE_qFec  = zeros( NumSamples + 1, 3 );
%         RMSE_qFet  = zeros( NumSamples + 1, 3 );
%         RMSE_qMecp = zeros( NumSamples + 1, 3 );
%         RMSE_qMecc = zeros( NumSamples + 1, 3 );
%         RMSE_qMet  = zeros( NumSamples + 1, 3 );
%         RMSE_qMeb  = zeros( NumSamples + 1, 3 );        

        % -------------------------------------
        %     F O R W A R D    E U L E R
        % -------------------------------------
        disp( 'Forward Euler ' )
        QUAT_FE_C( 1, : ) = QUAT_0; 
        for k = 1 : NumSamples 
            % CONSTANT OMEGA
            p_c = p(k);
            q_c = q(k);
            r_c = r(k);

            % column vector
            quat_dot_c = QDot( p_c, q_c, r_c, QUAT_FE_C( k, : )'); 

            % row vector
            QUAT_FE_C( k + 1, : ) = FwdEuler( quat_dot_c', SampleT, QUAT_FE_C( k, : ) );
        end

        QUAT_FE_T( 1, : ) = QUAT_0; 
        for k = 1 : NumSamples
            % TRAPEZOIDAL AVERAGE OF OMEGA
            p_t = (1/2)*(p(k+1)+p(k));
            q_t = (1/2)*(q(k+1)+q(k));
            r_t = (1/2)*(r(k+1)+r(k));

            % column vector
            quat_dot_t = QDot( p_t, q_t, r_t, QUAT_FE_T( k, : )'); 

            % row vector
            QUAT_FE_T( k + 1, : ) = FwdEuler( quat_dot_t', SampleT, QUAT_FE_T( k, : ) );
        end

        % -------------------------------------
        % M A T R I X   E X P O N E N T I A L
        % -------------------------------------
        disp( 'Matrix Exponential  ' )
        QUAT_ME_cp( 1, : ) = QUAT_0;   
        for k = 1 : NumSamples
        %   USE CONSTANT Wn (previous)
            p_cp = p(k);
            q_cp = q(k);
            r_cp = r(k);
            QUAT_ME_cp( k + 1, : ) = QMe_Int( p_cp, q_cp, r_cp, QUAT_ME_cp( k, : )', SampleT ); 
        end

        QUAT_ME_cc( 1, : ) = QUAT_0;   % IC1
        for k = 1 : NumSamples
        %   USE CONSTANT Wn+1 (current)
            p_cc = p(k+1);
            q_cc = q(k+1);
            r_cc = r(k+1);
            QUAT_ME_cc( k + 1, : ) = QMe_Int( p_cc, q_cc, r_cc, QUAT_ME_cc( k, : )', SampleT ); 
        end

        QUAT_ME_t( 1, : ) = QUAT_0;   % IC1
        for k = 1 : NumSamples 
        %   USE TRAPEZOIDAL AVG:  (1/2)*(Wn+1 + Wn)
            p_t = (1/2)*(p(k+1)+p(k));
            q_t = (1/2)*(q(k+1)+q(k));
            r_t = (1/2)*(r(k+1)+r(k));
            QUAT_ME_t( k + 1 , : ) = QMe_Int( p_t, q_t, r_t, QUAT_ME_t( k, : )', SampleT ); 
        end

        QUAT_ME_b( 1, : ) = QUAT_0;   % IC1
        % For the Omega_bar approach, we need two initial conditions due
        % to the central finite differencing method. Otherwise, QME(2,:) will
        % be zero.  Use the standard setup of instantaneous omega(tn).  
        % IC2:
        QUAT_ME_b( 2, : ) = QMe_Int( p( 2 ), q( 2 ), r( 2 ), QUAT_ME_b( 1, : )', SampleT ); 
        for k = 2 : NumSamples 
        %    if( k < NumSamples + 1 )
                OxNext = pqr2OmegaX_quat( p( k + 1 ), q( k + 1 ), r( k + 1 ));
                OxCurr = pqr2OmegaX_quat( p( k     ), q( k     ), r( k     ));
                OxPrev = pqr2OmegaX_quat( p( k - 1 ), q( k - 1 ), r( k - 1 ));

                Odot  = OmegaDot( OxNext, OxPrev, SampleT );
                Oddot = OmegaDDot( OxNext, OxCurr, OxPrev, SampleT );
                OmBar = OmegaBar( OxCurr, Odot, Oddot, SampleT );

                QUAT_ME_b( k + 1, : ) = QMe_Int2( OmBar, QUAT_ME_b( k, : )', SampleT );    
        %    end
        end

        E_q_fe_c  = Quat2Euler( QUAT_FE_C ); % each row = 1 timestep of [ phi theta psi ]
        E_q_fe_t  = Quat2Euler( QUAT_FE_T );
        E_q_me_cp = Quat2Euler( QUAT_ME_cp );
        E_q_me_cc = Quat2Euler( QUAT_ME_cc );
        E_q_me_t  = Quat2Euler( QUAT_ME_t );
        E_q_me_b  = Quat2Euler( QUAT_ME_b );

        if( idx == RUN_CLEAN )
            cE_q_fe_c( :, : )  = E_q_fe_c( :, : );
            cE_q_fe_t( :, : )  = E_q_fe_t( :, : );
            cE_q_me_cp( :, : ) = E_q_me_cp( :, : );
            cE_q_me_cc( :, : ) = E_q_me_cc( :, : );
            cE_q_me_t( :, : )  = E_q_me_t( :, : );
            cE_q_me_b( :, : )  = E_q_me_b( :, : );
        else % Noise testing!
        
            % CALCULATE RMSE STATS 
            % Calculate residuals - Error from 
            % Algo_Noise - Algo_Clean
            qerr_Fe_c   = E_q_fe_c  - cE_q_fe_c;  
            qerr_Fe_t   = E_q_fe_t  - cE_q_fe_t;
            qerr_Me_cp  = E_q_me_cp - cE_q_me_cp;  
            qerr_Me_cc  = E_q_me_cc - cE_q_me_cc;  
            qerr_Me_t   = E_q_me_t  - cE_q_me_t;  
            qerr_Me_b   = E_q_me_b  - cE_q_me_b;  

            % Square residuals
            for rdx = 1 : NumSamples + 1
                for edx = 1 : 3
                    RMSE_qFec( rdx, edx )  = ( (qerr_Fe_c(  rdx, edx ))^2 ); 
                    RMSE_qFet( rdx, edx )  = ( (qerr_Fe_t(  rdx, edx ))^2 ); 
                    RMSE_qMecp( rdx, edx ) = ( (qerr_Me_cp( rdx, edx ))^2 ); 
                    RMSE_qMecc( rdx, edx ) = ( (qerr_Me_cc( rdx, edx ))^2 ); 
                    RMSE_qMet( rdx, edx )  = ( (qerr_Me_t(  rdx, edx ))^2 ); 
                    RMSE_qMeb( rdx, edx )  = ( (qerr_Me_b(  rdx, edx ))^2 ); 
                end
            end

            % Append squared residuals to master array
            mRMSE_qFec  = vertcat( mRMSE_qFec,  RMSE_qFec) ;
            mRMSE_qFet  = vertcat( mRMSE_qFet,  RMSE_qFet) ;
            mRMSE_qMecp = vertcat( mRMSE_qMecp, RMSE_qMecp) ;
            mRMSE_qMecc = vertcat( mRMSE_qMecc, RMSE_qMecc) ;
            mRMSE_qMet  = vertcat( mRMSE_qMet,  RMSE_qMet) ;
            mRMSE_qMeb  = vertcat( mRMSE_qMeb,  RMSE_qMeb) ;
            
            % Append residuals to master array
            mDiff_qFec  = vertcat( mDiff_qFec , radtodeg(qerr_Fe_c ));
            mDiff_qFet  = vertcat( mDiff_qFet , radtodeg(qerr_Fe_t ));
            mDiff_qMecp = vertcat( mDiff_qMecp , radtodeg(qerr_Me_cp ));
            mDiff_qMecc = vertcat( mDiff_qMecc , radtodeg(qerr_Me_cc ));
            mDiff_qMet  = vertcat( mDiff_qMet , radtodeg(qerr_Me_t ));
            mDiff_qMeb  = vertcat( mDiff_qMeb , radtodeg(qerr_Me_b ));

        end % if run_clean 
    end % quat_test    

    %% ======================================
    %          E I G E N - A X I S   
    % ======================================

    if( AA_TEST == 1 )
        disp( 'ANGLE-AXIS IN PROGRESS:  ----------------- ' )
        % PREALLOCATE MEMORY
        Phi_fe_c  = zeros( NumSamples + 1, 1 );
        Axis_fe_c = zeros( NumSamples + 1, 3 );

        Phi_fe_t  = zeros( NumSamples + 1, 1 );
        Axis_fe_t = zeros( NumSamples + 1, 3 );

        E_a_fe_c = zeros( NumSamples + 1, 3 );
        E_a_fe_t = zeros( NumSamples + 1, 3 );

        R_fec = zeros( 3, 3, NumSamples + 1 );
        R_fet = zeros( 3, 3, NumSamples + 1 );
        
        RMSE_aFec  = zeros( NumSamples + 1, 3 );
        RMSE_aFet  = zeros( NumSamples + 1, 3 );
        

        % -------------------------------------
        %     F O R W A R D    E U L E R
        % -------------------------------------
        disp( 'Forward Euler ' )
        Phi_fe_c( 1 ) = aPhi_IC; 
        Axis_fe_c( 1, : ) = aAxis_IC; 
        for k = 1 : NumSamples 
            % CONSTANT OMEGA
                Om = [ p(k); q(k); r(k ) ];
                [ phiDot aDot ] = AngAxDot2( Om, Phi_fe_c( k ), Axis_fe_c( k, : )); 
                Phi_fe_c( k + 1 ) = FwdEuler( phiDot, SampleT, Phi_fe_c( k ) );
                Axis_fe_c( k + 1, : ) = FwdEuler( aDot, SampleT, Axis_fe_c( k, : ) ); 
        end

        Phi_fe_t( 1 ) = aPhi_IC; 
        Axis_fe_t( 1, : ) = aAxis_IC; 
        for k = 1 : NumSamples 
            % TRAPEZOIDAL AVERAGE OF OMEGA
                p_t = (1/2)*( p(k+1) + p(k) );
                q_t = (1/2)*( q(k+1) + q(k) );
                r_t = (1/2)*( r(k+1) + r(k) );
                Om = [ p_t; q_t; r_t ];

                [ phiDot aDot ] = AngAxDot2( Om, Phi_fe_t( k ), Axis_fe_t( k, : )); 
                Phi_fe_t( k + 1 ) = FwdEuler( phiDot, SampleT, Phi_fe_t( k ) );
                Axis_fe_t( k + 1, : ) = FwdEuler( aDot, SampleT, Axis_fe_t( k, : ) ); 
        end

        % Conversion back to Euler angles
        for k = 1 : NumSamples + 1
            R_fec( :, :, k ) = AngAx2DCM( Phi_fe_c( k ), Axis_fe_c( k, 1 ), ...
               Axis_fe_c( k, 2 ), Axis_fe_c( k, 3 ));
            E_a_fe_c( k, : ) = DCM2Euler( R_fec( :, :, k ) );

            R_fet( :, :, k ) = AngAx2DCM( Phi_fe_t( k ), Axis_fe_t( k, 1 ), ...
               Axis_fe_t( k, 2 ), Axis_fe_t( k, 3 ));
            E_a_fe_t( k, : ) = DCM2Euler( R_fet( :, :, k ) );
        end

        if( idx == RUN_CLEAN )
            cE_a_fe_c( :, : )  = E_a_fe_c( :, : );
            cE_a_fe_t( :, : )  = E_a_fe_t( :, : );

        else % Noise testing!        
            % CALCULATE RMSE STATS 
            % Calculate residuals - Error from 
            % Algo_Noise - Algo_Clean
            Aerr_Fe_c   = E_a_fe_c  - cE_a_fe_c;  
            Aerr_Fe_t   = E_a_fe_t  - cE_a_fe_t;

            % Square residuals
            for rdx = 1 : NumSamples + 1
                for edx = 1 : 3
                    RMSE_aFec( rdx, edx )  = ( (Aerr_Fe_c(  rdx, edx ))^2 ); 
                    RMSE_aFet( rdx, edx )  = ( (Aerr_Fe_t(  rdx, edx ))^2 ); 
                end
            end

            % Append squared residuals to master array
            mRMSE_aFec  = vertcat( mRMSE_aFec, RMSE_aFec) ;
            mRMSE_aFet  = vertcat( mRMSE_aFet, RMSE_aFet) ;

            % Append residuals to master array            
            mDiff_aFec  = vertcat( mDiff_aFec, radtodeg(Aerr_Fe_c ));
            mDiff_aFet  = vertcat( mDiff_aFet, radtodeg(Aerr_Fe_t ));            

        end % if run_clean 
    end %% aa_test

    % ++++++++++++++++++++++++++++++++++++++
    % ======================================
    %          P L O T   R E S U L T S 
    % ======================================
    % ++++++++++++++++++++++++++++++++++++++

%%
    % ----------------------------------
    %               D C M
    % ----------------------------------
    if( DCM_TEST == 1 )
        parm_dir = '\DCM';

        % PLOT ANGLE RESPONSE
        % phi
        h_dcm_phi = figure;
        plot( t', radtodeg(Phi_T), 'k', 'linewidth',2);
        hold on
        plot( t', radtodeg(E_r_fe_c(  :, C_phi )), 'b--', 'linewidth',2);
        plot( t', radtodeg(E_r_fe_t(  :, C_phi )), 'b-.', 'linewidth',2); 
        plot( t', radtodeg(E_r_me_cc( :, C_phi )), 'r--', 'linewidth',2);
        plot( t', radtodeg(E_r_me_cp( :, C_phi )), 'r-.', 'linewidth',2); 
        plot( t', radtodeg(E_r_me_t(  :, C_phi )), 'g--', 'linewidth',2);
        plot( t', radtodeg(E_r_me_b(  :, C_phi )), 'g-.', 'linewidth',2);
        legend( 'True', 'Fwd Euler Const' , 'Fwd Euler Trap', 'MExp Const Curr', ...
            'MExp Const Prev', 'MExp Trap', 'MExp OmBar', 'Location', 'SouthWest')
        title( strcat( 'DCM - ', 'Phi (\phi) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        savefile = strcat( 'dat\', dir, parm_dir, '\Phi', str_run_mode, '.fig' );
        saveas( h_dcm_phi, savefile );

        % theta
        h_dcm_theta = figure;
        plot( t', radtodeg(Theta_T), 'k', 'linewidth',2);
        hold on
        plot( t', radtodeg(E_r_fe_c(  :, C_theta )), 'b--', 'linewidth',2); 
        plot( t', radtodeg(E_r_fe_t(  :, C_theta )), 'b-.', 'linewidth',2);
        plot( t', radtodeg(E_r_me_cc( :, C_theta )), 'r--' , 'linewidth',2);
        plot( t', radtodeg(E_r_me_cp( :, C_theta )), 'r-.', 'linewidth',2);
        plot( t', radtodeg(E_r_me_t(  :, C_theta )), 'g--', 'linewidth',2);
        plot( t', radtodeg(E_r_me_b(  :, C_theta )), 'g-.', 'linewidth',2);
        legend( 'True', 'Fwd Euler Const' , 'Fwd Euler Trap', 'MExp Const Curr', ...
            'MExp Const Prev', 'MExp Trap', 'MExp OmBar' )
        title( strcat('DCM - ', 'Theta (\theta) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        savefile = strcat( 'dat\', dir, parm_dir, '\Theta', str_run_mode, '.fig' );
        saveas( h_dcm_theta, savefile );        

        % psi
        h_dcm_psi = figure;
        plot( t', radtodeg(Psi_T), 'k', 'linewidth',2);
        hold on
        plot( t', radtodeg(E_r_fe_c(  :, C_psi )), 'b--', 'linewidth',2);
        plot( t', radtodeg(E_r_fe_t(  :, C_psi )), 'b-.', 'linewidth',2);
        plot( t', radtodeg(E_r_me_cc( :, C_psi )), 'r--', 'linewidth',2);
        plot( t', radtodeg(E_r_me_cp( :, C_psi )), 'r-.', 'linewidth',2);
        plot( t', radtodeg(E_r_me_t(  :, C_psi )), 'g--', 'linewidth',2);
        plot( t', radtodeg(E_r_me_b(  :, C_psi )), 'g-.', 'linewidth',2);
        legend( 'True', 'Fwd Euler Const' , 'Fwd Euler Trap', 'MExp Const Curr', ...
            'MExp Const Prev', 'MExp Trap', 'MExp OmBar' )
        title( strcat('DCM - ', 'Psi (\psi) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        savefile = strcat( 'dat\', dir, parm_dir, '\Psi', str_run_mode, '.fig' );
        saveas( h_dcm_psi, savefile );        
        
        
        % PLOT ERRORS
        % Phi
%         h_dcm_phi_err = figure;
%         plot( t, radtodeg(Rerr_Fe_c(:,C_phi)), 'b--', 'linewidth',2);
%         hold on
%         plot( t, radtodeg(Rerr_Fe_t(:,C_phi)), 'b-.', 'linewidth',2);
%         plot( t, radtodeg(Rerr_Me_cc(:,C_phi)), 'r--', 'linewidth',2);
%         plot( t, radtodeg(Rerr_Me_cp(:,C_phi)), 'r-.', 'linewidth',2);
%         plot( t, radtodeg(Rerr_Me_t(:,C_phi)), 'g--', 'linewidth',2);
%         plot( t, radtodeg(Rerr_Me_b(:,C_phi)), 'g-.', 'linewidth',2);
%         legend( 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
%             'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
%         title( 'DCM - Phi Error' )
%         xlabel( 'Time (s)')
%         ylabel( 'Error (deg)' )
%         grid on
%         savefile = strcat( 'dat\', dir, parm_dir, '\Phi_Err', str_run_mode, '.fig' );
%         saveas( h_dcm_phi_err, savefile );        
% 
%         % Theta
%         h_dcm_theta_err = figure;
%         plot( t, radtodeg(Rerr_Fe_c(:,C_theta)), 'b--', 'linewidth',2);
%         hold on
%         plot( t, radtodeg(Rerr_Fe_t(:,C_theta)), 'b-.', 'linewidth',2);
%         plot( t, radtodeg(Rerr_Me_cc(:,C_theta)), 'r--', 'linewidth',2);
%         plot( t, radtodeg(Rerr_Me_cp(:,C_theta)), 'r-.', 'linewidth',2);
%         plot( t, radtodeg(Rerr_Me_t(:,C_theta)), 'g--', 'linewidth',2);
%         plot( t, radtodeg(Rerr_Me_b(:,C_theta)), 'g-.', 'linewidth',2);
%         legend( 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
%             'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
%         title( 'DCM - Theta Error' )
%         xlabel( 'Time (s)')
%         ylabel( 'Error (deg)' )
%         grid on
%         savefile = strcat( 'dat\', dir, parm_dir, '\Theta_Err', str_run_mode, '.fig' );
%         saveas( h_dcm_theta_err, savefile );        
% 
%         % Psi
%         h_dcm_psi_err = figure;
%         plot( t, radtodeg(Rerr_Fe_c(:,C_psi)), 'b--', 'linewidth',2);
%         hold on
%         plot( t, radtodeg(Rerr_Fe_t(:,C_psi)), 'b-.', 'linewidth',2);
%         plot( t, radtodeg(Rerr_Me_cc(:,C_psi)), 'r--', 'linewidth',2);
%         plot( t, radtodeg(Rerr_Me_cp(:,C_psi)), 'r-.', 'linewidth',2);
%         plot( t, radtodeg(Rerr_Me_t(:,C_psi)), 'g--', 'linewidth',2);
%         plot( t, radtodeg(Rerr_Me_b(:,C_psi)), 'g-.', 'linewidth',2);
%         legend( 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
%             'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
%         title( 'DCM - Psi Error' )
%         xlabel( 'Time (s)')
%         ylabel( 'Error (deg)' )
%         grid on
%         savefile = strcat( 'dat\', dir, parm_dir, '\Psi_Err', str_run_mode, '.fig' );
%         saveas( h_dcm_psi_err, savefile );   
         
%         
%         if( idx ~= RUN_CLEAN )
%             % DIFFERENCE SIGNALS FROM NOISE TESTS
%             % Phi
%             h_dcm_phi_diff = figure;
%             plot( t', RK4_Err( :, C_phi), 'k' )
%             hold on
%             plot( t', Rerr_Fe_c(  :, C_phi ), 'b--' )
%             plot( t', Rerr_Fe_t(  :, C_phi ), 'b-.' )
%             plot( t', Rerr_Me_cc( :, C_phi ), 'r--' )
%             plot( t', Rerr_Me_cp( :, C_phi ), 'r-.' )
%             plot( t', Rerr_Me_t(  :, C_phi ), 'g--' )
%             plot( t', Rerr_Me_b(  :, C_phi ), 'g-.' )
%             legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
%                 'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
%             title( 'DCM - Phi Residual Signal' )
%             xlabel( 'Time (s)')
%             ylabel( 'Angle (deg)' )
%             grid on
%             savefile = strcat( 'dat\', dir, parm_dir, '\Phi_Diff', str_run_mode, '.fig' );
%             saveas( h_dcm_phi_diff, savefile );        
% 
%             % Theta
%             h_dcm_theta_diff = figure;
%             plot( t', RK4_Err( :, C_theta), 'k' )
%             hold on
%             plot( t', Rerr_Fe_c(  :, C_theta ), 'b--' )
%             plot( t', Rerr_Fe_t(  :, C_theta ), 'b-.' )
%             plot( t', Rerr_Me_cc( :, C_theta ), 'r--' )
%             plot( t', Rerr_Me_cp( :, C_theta ), 'r-.' )
%             plot( t', Rerr_Me_t(  :, C_theta ), 'g--' )
%             plot( t', Rerr_Me_b(  :, C_theta ), 'g-.' )
%             legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
%                 'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
%             title( 'DCM - Theta Residual Signal' )
%             xlabel( 'Time (s)')
%             ylabel( 'Angle (deg)' )
%             grid on
%             savefile = strcat( 'dat\', dir, parm_dir, '\Theta_Diff', str_run_mode, '.fig' );
%             saveas( h_dcm_theta_diff, savefile ); 
% 
%             % Psi
%             h_dcm_psi_diff = figure;
%             plot( t', RK4_Err( :, C_psi), 'k' )
%             hold on
%             plot( t', Rerr_Fe_c(  :, C_psi ), 'b--' )
%             plot( t', Rerr_Fe_t(  :, C_psi ), 'b-.' )
%             plot( t', Rerr_Me_cc( :, C_psi ), 'r--' )
%             plot( t', Rerr_Me_cp( :, C_psi ), 'r-.' )
%             plot( t', Rerr_Me_t(  :, C_psi ), 'g--' )
%             plot( t', Rerr_Me_b(  :, C_psi ), 'g-.' )
%             legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
%                 'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
%             title( 'DCM - Psi Residual Signal' )
%             xlabel( 'Time (s)')
%             ylabel( 'Angle (deg)' )
%             grid on
%             savefile = strcat( 'dat\', dir, parm_dir, '\Psi_Diff', str_run_mode, '.fig' );
%             saveas( h_dcm_psi_diff, savefile ); 
%         end
    end % DCM_TEST
%% 
%     % ----------------------------------
%     %      E U L E R   A N G L E S
%     % ----------------------------------
    if( EULER_TEST == 1 )
         parm_dir = '\EULER';

        % phi
        h_euler_phi = figure;
        plot( t', radtodeg(Phi_T), 'k', 'linewidth', 2 )
        hold on
        plot( t', radtodeg(E_e_fe_c(  :, C_phi )), 'g', 'linewidth', 2 ) 
        plot( t', radtodeg(E_e_fe_t(  :, C_phi )), 'r' , 'linewidth', 2 )
        legend( 'True', 'Fwd Euler Const', 'Fwd Euler Avg' )
        title( strcat( 'EULER - ', 'Phi (\phi) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        savefile = strcat( 'dat\', dir, parm_dir, '\Phi', str_run_mode, '.fig' );
        saveas( h_euler_phi, savefile );        

        % theta
        h_euler_theta = figure;
        plot( t', radtodeg(Theta_T), 'k', 'linewidth', 2 )
        hold on
        plot( t', radtodeg(E_e_fe_c(  :, C_theta )), 'g', 'linewidth', 2 )
        plot( t', radtodeg(E_e_fe_t(  :, C_theta )), 'r', 'linewidth', 2 )
        legend( 'True', 'Fwd Euler Const' , 'Fwd Euler Avg' )
        title( strcat('EULER - ', 'Theta (\theta) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        savefile = strcat( 'dat\', dir, parm_dir, '\Theta', str_run_mode, '.fig' );
        saveas( h_euler_theta, savefile );        

        % psi
        h_euler_psi = figure;
        plot( t', radtodeg(Psi_T), 'k', 'linewidth', 2 )
        hold on
        plot( t', radtodeg(E_e_fe_c(  :, C_psi )), 'g', 'linewidth', 2 )
        plot( t', radtodeg(E_e_fe_t(  :, C_psi )), 'r', 'linewidth', 2 ) 
        legend( 'True', 'Fwd Euler Const' , 'Fwd Euler Avg' )
        title( strcat('EULER - ', 'Psi (\psi) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        savefile = strcat( 'dat\', dir, parm_dir, '\Psi', str_run_mode, '.fig' );
        saveas( h_euler_psi, savefile );        
         
         
         % 
%         % Error stats
%         Eerr_Fe_c   = E_e_fe_c - E_q ;  
%         Eerr_Fe_t   = E_e_fe_t - E_q ;
%         
%         % time history of rms errors
%         for rdx = 1 : NumSamples + 1
%             for edx = 1 : 3
%                 RMSE_eFec( rdx, edx )  = sqrt( (Eerr_Fe_c(  rdx, edx ))^2 ); 
%                 RMSE_eFet( rdx, edx )  = sqrt( (Eerr_Fe_t(  rdx, edx ))^2 ); 
%             end
%         end
%         
%         % root mean squared error
%         disp( 'RMSE point values: ' )
%         pRMSE_eFec = mean(RMSE_eFec)
%         pRMSE_eFet = mean(RMSE_eFet)
%         
%         xlswrite( 'eulerRMSE.xlsx', [pRMSE_eFec; pRMSE_eFet ] );
%         
%         % dump histograms 
%         numBins = 15;
%         
%         figure
%         subplot 131
%         hist( RMSE_eFec( :, 1 ), numBins )
%         title( 'EULER / Roll - RMSE Distribution for Forward Euler, Const Prev' )
%         xlabel( 'RMSE (rad)')
%         ylabel( 'Num Samples')
%         subplot 132
%         hist( RMSE_eFec( :, 2 ), numBins )
%         title( 'EULER / Pitch - RMSE Distribution for Forward Euler, Const Prev' ) 
%         xlabel( 'RMSE (rad)')
%         ylabel( 'Num Samples')
%         subplot 133
%         hist( RMSE_eFec( :, 3 ), numBins )
%         title( 'EULER / Yaw - RMSE Distribution for Forward Euler, Const Prev' )   
%         xlabel( 'RMSE (rad)')
%         ylabel( 'Num Samples')        
%         
%         figure
%         subplot 131
%         hist( RMSE_eFet( :, 1 ), numBins )
%         title( 'EULER / Roll - RMSE Distribution for Forward Euler, Trap' )
%         xlabel( 'RMSE (rad)')
%         ylabel( 'Num Samples')        
%         subplot 132
%         hist( RMSE_eFet( :, 2 ), numBins )
%         title( 'EULER / Pitch - RMSE Distribution for Forward Euler, Trap' )
%         xlabel( 'RMSE (rad)')
%         ylabel( 'Num Samples')        
%         subplot 133
%         hist( RMSE_eFet( :, 3 ), numBins )
%         title( 'EULER / Yaw - RMSE Distribution for Forward Euler, Trap' )
%         xlabel( 'RMSE (rad)')
%         ylabel( 'Num Samples')        
%         
%         
% 
%         MEerr_Fe_c = mean( Eerr_Fe_c );
%         MEerr_Fe_t = mean( Eerr_Fe_t );
%         Euler_Err = [ MEerr_Fe_c ; MEerr_Fe_t ]
% 
%         stdEerr_Fe_c = std( Eerr_Fe_c );
%         stdEerr_Fe_t = std( Eerr_Fe_t );
%         stdEuler_Err = [ stdEerr_Fe_c ; stdEerr_Fe_t ]
%         
%         % Store to disk
%         savefile = strcat( 'dat\', dir, parm_dir, '\Error', str_run_mode, '.mat' );
%         save( savefile, 'Euler_Err', 'stdEuler_Err' );
% 
        % Phi
%         h_euler_phi_err = figure;
%         plot( t', radtodeg(Eerr_Fe_c(  :, C_phi )), 'g', 'Linewidth', 2 )
%         hold on
%         plot( t', radtodeg(Eerr_Fe_t(  :, C_phi )), 'r', 'Linewidth', 2 )
%         legend( 'Fwd Euler Const', 'Fwd Euler Avg' )
%         title( 'EULER - Phi Error' )
%         xlabel( 'Time (s)')
%         ylabel( 'Error (deg)' )
%         grid on
%         savefile = strcat( 'dat\', dir, parm_dir, '\Phi_Err', str_run_mode, '.fig' );
%         saveas( h_euler_phi_err, savefile );        
% 
%         % Theta
%         h_euler_theta_err = figure;
%         plot( t', radtodeg(Eerr_Fe_c(  :, C_theta )), 'g', 'Linewidth', 2 )
%         hold on
%         plot( t', radtodeg(Eerr_Fe_t(  :, C_theta )), 'r', 'Linewidth', 2 )
%         legend( 'Fwd Euler Const', 'Fwd Euler Avg' );
%         title( 'EULER - Theta Error' )
%         xlabel( 'Time (s)')
%         ylabel( 'Error (deg)' )
%         grid on
%         savefile = strcat( 'dat\', dir, parm_dir, '\Theta_Err', str_run_mode, '.fig' );
%         saveas( h_euler_theta_err, savefile );
%         
%         % Psi
%         h_euler_psi_err = figure;
%         plot( t', radtodeg(Eerr_Fe_c(  :, C_psi )), 'g', 'Linewidth', 2 )
%         hold on
%         plot( t', radtodeg(Eerr_Fe_t(  :, C_psi )), 'r', 'Linewidth', 2 )
%         legend( 'Fwd Euler Const', 'Fwd Euler Avg' );
%         title( 'EULER - Psi Error' )
%         xlabel( 'Time (s)')
%         ylabel( 'Error (deg)' )
%         grid on
%         savefile = strcat( 'dat\', dir, parm_dir, '\Psi_Err', str_run_mode, '.fig' );
%         saveas( h_euler_psi_err, savefile );  
%         
%         if( idx ~= RUN_CLEAN )
%             % DIFFERENCE SIGNALS FROM NOISE TESTS
%             % Phi
%             h_euler_phi_diff = figure;
%             plot( t', RK4_Err( :, C_phi), 'k' )
%             hold on
%             plot( t', Eerr_Fe_c(  :, C_phi ), 'g' )
%             plot( t', Eerr_Fe_t(  :, C_phi ), 'r' )
%             legend( 'True', 'Fwd Euler Const', 'Fwd Euler Avg')
%             title( 'EULER - Phi Residual Signal' )
%             xlabel( 'Time (s)')
%             ylabel( 'Angle (deg)' )
%             grid on
%             savefile = strcat( 'dat\', dir, parm_dir, '\Phi_Diff', str_run_mode, '.fig' );
%             saveas( h_euler_phi_diff, savefile );        
% 
%             % Theta
%             h_euler_theta_diff = figure;
%             plot( t', RK4_Err( :, C_theta), 'k' )
%             hold on
%             plot( t', Eerr_Fe_c(  :, C_theta ), 'g' )
%             plot( t', Eerr_Fe_t(  :, C_theta ), 'r' )
%             legend( 'True', 'Fwd Euler Const', 'Fwd Euler Avg')
%             title( 'EULER - Theta Residual Signal' )
%             xlabel( 'Time (s)')
%             ylabel( 'Angle (deg)' )
%             grid on
%             savefile = strcat( 'dat\', dir, parm_dir, '\Theta_Diff', str_run_mode, '.fig' );
%             saveas( h_euler_theta_diff, savefile ); 
% 
%             % Psi
%             h_euler_psi_diff = figure;
%             plot( t', RK4_Err( :, C_psi), 'k' )
%             hold on
%             plot( t', Eerr_Fe_c(  :, C_psi ), 'g' )
%             plot( t', Eerr_Fe_t(  :, C_psi ), 'r' )
% 
%             legend( 'True', 'Fwd Euler Const', 'Fwd Euler Avg')
%             title( 'EULER - Psi Residual Signal' )
%             xlabel( 'Time (s)')
%             ylabel( 'Angle (deg)' )
%             grid on
%             savefile = strcat( 'dat\', dir, parm_dir, '\Psi_Diff', str_run_mode, '.fig' );
%             saveas( h_euler_psi_diff, savefile ); 
%         end
    end
%% 
%     % ----------------------------------
%     %       Q U A T E R N I O N S  
%     % ----------------------------------
    if( QUAT_TEST == 1 )
        parm_dir = '\QUAT';
        
        % phi
        h_quat_phi = figure;
        plot( t', radtodeg(Phi_T), 'k', 'linewidth',2);
        hold on
        plot( t, radtodeg(E_q_fe_c(:,C_phi)), 'b--', 'linewidth',2); 
        plot( t, radtodeg(E_q_fe_t(:,C_phi)), 'b-.', 'linewidth',2);
        plot( t, radtodeg(E_q_me_cc(:,C_phi)), 'r--', 'linewidth',2);
        plot( t, radtodeg(E_q_me_cp(:,C_phi)), 'r-.', 'linewidth',2);
        plot( t, radtodeg(E_q_me_t(:,C_phi)), 'g--', 'linewidth',2);
        plot( t, radtodeg(E_q_me_b(:,C_phi)), 'g-.', 'linewidth',2);
        legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
            'MExp Const Prev', 'MExp Trap', 'MExp OmBar', 'Location', 'SouthWest')
        title( 'QUAT - Phi (\phi)')
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Phi', str_run_mode, '.fig' );
        saveas( h_quat_phi, savefile );        

        % theta
        h_quat_theta = figure;
        plot( t', radtodeg(Theta_T), 'k', 'linewidth',2);
        hold on
        plot( t, radtodeg(E_q_fe_c(:,C_theta)), 'b--', 'linewidth',2); 
        plot( t, radtodeg(E_q_fe_t(:,C_theta)), 'b-.', 'linewidth',2);
        plot( t, radtodeg(E_q_me_cc(:,C_theta)), 'r--', 'linewidth',2);
        plot( t, radtodeg(E_q_me_cp(:,C_theta)), 'r-.', 'linewidth',2);
        plot( t, radtodeg(E_q_me_t(:,C_theta)), 'g--', 'linewidth',2);
        plot( t, radtodeg(E_q_me_b(:,C_theta)), 'g-.', 'linewidth',2);
        legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
            'MExp Const Prev', 'MExp Trap', 'MExp OmBar', 'Location', 'SouthWest')
        title( 'QUAT - Theta (\theta)')
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Theta', str_run_mode, '.fig' );
        saveas( h_quat_theta, savefile );        

        % psi
        h_quat_psi = figure;
        plot( t', radtodeg(Psi_T), 'k', 'linewidth',2);
        hold on
        plot( t, radtodeg(E_q_fe_c(:,C_psi)), 'b--', 'linewidth',2);  
        plot( t, radtodeg(E_q_fe_t(:,C_psi)), 'b-.', 'linewidth',2); 
        plot( t, radtodeg(E_q_me_cc(:,C_psi)), 'r--', 'linewidth',2);
        plot( t, radtodeg(E_q_me_cp(:,C_psi)), 'r-.', 'linewidth',2);
        plot( t, radtodeg(E_q_me_t(:,C_psi)), 'g--', 'linewidth',2);
        plot( t, radtodeg(E_q_me_b(:,C_psi)), 'g-.', 'linewidth',2);
        legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
            'MExp Const Prev', 'MExp Trap', 'MExp OmBar', 'Location', 'SouthWest')
        title( 'QUAT - Psi (\psi)' )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Psi', str_run_mode, '.fig' );
        saveas( h_quat_psi, savefile );        
        
        % ERROR PLOTS
        % Phi
%         h_quat_phi_err = figure;
%         plot( t, radtodeg(qerr_Fe_c(:,C_phi)), 'b--', 'linewidth',2);
%         hold on
%         plot( t, radtodeg(qerr_Fe_t(:,C_phi)), 'b-.', 'linewidth',2);
%         plot( t, radtodeg(qerr_Me_cc(:,C_phi)), 'r--', 'linewidth',2);
%         plot( t, radtodeg(qerr_Me_cp(:,C_phi)), 'r-.', 'linewidth',2);
%         plot( t, radtodeg(qerr_Me_t(:,C_phi)), 'g--', 'linewidth',2);
%         plot( t, radtodeg(qerr_Me_b(:,C_phi)), 'g-.', 'linewidth',2);
%         legend( 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
%             'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
%         title( 'QUAT - Phi Error' )
%         xlabel( 'Time (s)')
%         ylabel( 'Error (deg)' )
%         grid on
%         savefile = strcat( 'dat\', dir, parm_dir, '\Phi_Err', str_run_mode, '.fig' );
%         saveas( h_quat_phi_err, savefile );        
% 
%         % Theta
%         h_quat_theta_err = figure;
%         plot( t, radtodeg(qerr_Fe_c(:,C_theta)), 'b--', 'linewidth',2);
%         hold on
%         plot( t, radtodeg(qerr_Fe_t(:,C_theta)), 'b-.', 'linewidth',2);
%         plot( t, radtodeg(qerr_Me_cc(:,C_theta)), 'r--', 'linewidth',2);
%         plot( t, radtodeg(qerr_Me_cp(:,C_theta)), 'r-.', 'linewidth',2);
%         plot( t, radtodeg(qerr_Me_t(:,C_theta)), 'g--', 'linewidth',2);
%         plot( t, radtodeg(qerr_Me_b(:,C_theta)), 'g-.', 'linewidth',2);
%         legend( 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
%             'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
%         title( 'QUAT - Theta Error' )
%         xlabel( 'Time (s)')
%         ylabel( 'Error (deg)' )
%         grid on
%         savefile = strcat( 'dat\', dir, parm_dir, '\Theta_Err', str_run_mode, '.fig' );
%         saveas( h_quat_theta_err, savefile );        
% 
%         % Psi
%         h_quat_psi_err = figure;
%         plot( t, radtodeg(qerr_Fe_c(:,C_psi)), 'b--', 'linewidth',2);
%         hold on
%         plot( t, radtodeg(qerr_Fe_t(:,C_psi)), 'b-.', 'linewidth',2);
%         plot( t, radtodeg(qerr_Me_cc(:,C_psi)), 'r--', 'linewidth',2);
%         plot( t, radtodeg(qerr_Me_cp(:,C_psi)), 'r-.', 'linewidth',2);
%         plot( t, radtodeg(qerr_Me_t(:,C_psi)), 'g--', 'linewidth',2);
%         plot( t, radtodeg(qerr_Me_b(:,C_psi)), 'g-.', 'linewidth',2);
%         legend( 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
%             'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
%         title( 'QUAT - Psi Error' )
%         xlabel( 'Time (s)')
%         ylabel( 'Error (deg)' )
%         grid on
%         savefile = strcat( 'dat\', dir, parm_dir, '\Psi_Err', str_run_mode, '.fig' );
%         saveas( h_quat_psi_err, savefile );   
%         
%         if( idx ~= RUN_CLEAN )
%             % DIFFERENCE SIGNALS FROM NOISE TESTS
%             % Phi
%             h_quat_phi_diff = figure;
%             plot( t', RK4_Err( :, C_phi), 'k' )
%             hold on
%             plot( t', qerr_Fe_c(  :, C_phi ), 'b--' )
%             plot( t', qerr_Fe_t(  :, C_phi ), 'b-.' )
%             plot( t', qerr_Me_cc( :, C_phi ), 'r--' )
%             plot( t', qerr_Me_cp( :, C_phi ), 'r-.' )
%             plot( t', qerr_Me_t(  :, C_phi ), 'g--' )
%             plot( t', qerr_Me_b(  :, C_phi ), 'g-.' )
%             legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
%                 'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
%             title( 'QUAT - Phi Difference Signal' )
%             xlabel( 'Time (s)')
%             ylabel( 'Angle (deg)' )
%             grid on
%             savefile = strcat( 'dat\', dir, parm_dir, '\Phi_Diff', str_run_mode, '.fig' );
%             saveas( h_quat_phi_diff, savefile );        
% 
%             % Theta
%             h_quat_theta_diff = figure;
%             plot( t', RK4_Err( :, C_theta), 'k' )
%             hold on
%             plot( t', qerr_Fe_c(  :, C_theta ), 'b--' )
%             plot( t', qerr_Fe_t(  :, C_theta ), 'b-.' )
%             plot( t', qerr_Me_cc( :, C_theta ), 'r--' )
%             plot( t', qerr_Me_cp( :, C_theta ), 'r-.' )
%             plot( t', qerr_Me_t(  :, C_theta ), 'g--' )
%             plot( t', qerr_Me_b(  :, C_theta ), 'g-.' )
%             legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
%                 'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
%             title( 'QUAT - Theta Difference Signal' )
%             xlabel( 'Time (s)')
%             ylabel( 'Angle (deg)' )
%             grid on
%             savefile = strcat( 'dat\', dir, parm_dir, '\Theta_Diff', str_run_mode, '.fig' );
%             saveas( h_quat_theta_diff, savefile ); 
% 
%             % Psi
%             h_quat_psi_diff = figure;
%             plot( t', RK4_Err( :, C_psi), 'k' )
%             hold on
%             plot( t', qerr_Fe_c(  :, C_psi ), 'b--' )
%             plot( t', qerr_Fe_t(  :, C_psi ), 'b-.' )
%             plot( t', qerr_Me_cc( :, C_psi ), 'r--' )
%             plot( t', qerr_Me_cp( :, C_psi ), 'r-.' )
%             plot( t', qerr_Me_t(  :, C_psi ), 'g--' )
%             plot( t', qerr_Me_b(  :, C_psi ), 'g-.' )
%             legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
%                 'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
%             title( 'QUAT - Psi Difference Signal' )
%             xlabel( 'Time (s)')
%             ylabel( 'Angle (deg)' )
%             grid on
%             savefile = strcat( 'dat\', dir, parm_dir, '\Psi_Diff', str_run_mode, '.fig' );
%             saveas( h_quat_psi_diff, savefile ); 
%         end
    end
%% 
%     % ----------------------------------
%     %       E I G E N - A X I S 
%     % ----------------------------------
    if( AA_TEST == 1 )
         parm_dir = '\AA';
         
        % PLOT ANGLES
        % phi
        h_aa_phi = figure;
        plot( t', radtodeg(Phi_T), 'k', 'linewidth', 2 )
        hold on
        plot( t', radtodeg(E_a_fe_c(  :, C_phi )), 'g', 'linewidth', 2 ) 
        plot( t', radtodeg(E_a_fe_t(  :, C_phi )), 'r' , 'linewidth', 2 )
        legend( 'True', 'Fwd Euler Const' , 'Fwd Euler Trap' ); 
        title( strcat('ANG-AX:  Phi (\phi) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        savefile = strcat( 'dat\', dir, parm_dir, '\Phi', str_run_mode, '.fig' );
        saveas( h_aa_phi, savefile );        

        % theta
        h_aa_theta = figure;
        plot( t', radtodeg(Theta_T), 'k', 'linewidth', 2 )
        hold on
        plot( t', radtodeg(E_a_fe_c(  :, C_theta )), 'g', 'linewidth', 2 ) 
        plot( t', radtodeg(E_a_fe_t(  :, C_theta )), 'r', 'linewidth', 2 )
        legend( 'True', 'Fwd Euler Const' , 'Fwd Euler Trap' )
        title( strcat('ANG-AX:  Theta (\theta) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        savefile = strcat( 'dat\', dir, parm_dir, '\Theta', str_run_mode, '.fig' );
        saveas( h_aa_theta, savefile );        

        % psi
        h_aa_psi = figure;
        plot( t', radtodeg(Psi_T), 'k' )
        hold on
        plot( t', radtodeg(E_a_fe_c(  :, C_psi )), 'g', 'linewidth', 2 ) 
        plot( t', radtodeg(E_a_fe_t(  :, C_psi )), 'r', 'linewidth', 2 )
        legend( 'True', 'Fwd Euler Const'  , 'Fwd Euler Trap' );
        title( strcat('ANG-AX:  Psi (\psi) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        savefile = strcat( 'dat\', dir, parm_dir, '\Psi', str_run_mode, '.fig' );
        saveas( h_aa_psi, savefile );        
% 
%         aaErr_Fe_c   = E_a_fe_c - E_q;
%         aaErr_Fe_t   = E_a_fe_t - E_q;
%         
%         % time history of rms errors
%         for rdx = 1 : NumSamples + 1
%             for edx = 1 : 3
%                 RMSE_aFec( rdx, edx )  = sqrt( (aaErr_Fe_c(  rdx, edx ))^2 ); 
%                 RMSE_aFet( rdx, edx )  = sqrt( (aaErr_Fe_t(  rdx, edx ))^2 ); 
%             end
%         end
%         
%         % root mean squared error
%         disp( 'RMSE point values: ' )
%         pRMSE_aFec = mean(RMSE_aFec)
%         pRMSE_aFet = mean(RMSE_aFet)
%         
%         xlswrite( 'angaxRMSE.xlsx', [pRMSE_aFec; pRMSE_aFet ] );
%         
%         % dump histograms 
%         numBins = 15;
%         
%         figure
%         subplot 131
%         hist( RMSE_aFec( :, 1 ), numBins )
%         title( 'ANG-AX / Roll - RMSE Distribution for Forward Euler, Const Prev' )
%         xlabel( 'RMSE (rad)')
%         ylabel( 'Num Samples')
%         subplot 132
%         hist( RMSE_aFec( :, 2 ), numBins )
%         title( 'ANG-AX / Pitch - RMSE Distribution for Forward Euler, Const Prev' ) 
%         xlabel( 'RMSE (rad)')
%         ylabel( 'Num Samples')
%         subplot 133
%         hist( RMSE_aFec( :, 3 ), numBins )
%         title( 'ANG-AX / Yaw - RMSE Distribution for Forward Euler, Const Prev' )   
%         xlabel( 'RMSE (rad)')
%         ylabel( 'Num Samples')        
%         
%         figure
%         subplot 131
%         hist( RMSE_aFet( :, 1 ), numBins )
%         title( 'ANG-AX / Roll - RMSE Distribution for Forward Euler, Trap' )
%         xlabel( 'RMSE (rad)')
%         ylabel( 'Num Samples')        
%         subplot 132
%         hist( RMSE_aFet( :, 2 ), numBins )
%         title( 'ANG-AX / Pitch - RMSE Distribution for Forward Euler, Trap' )
%         xlabel( 'RMSE (rad)')
%         ylabel( 'Num Samples')        
%         subplot 133
%         hist( RMSE_aFet( :, 3 ), numBins )
%         title( 'ANG-AX / Yaw - RMSE Distribution for Forward Euler, Trap' )
%         xlabel( 'RMSE (rad)')
%         ylabel( 'Num Samples')        
        
        
% 
%         MaaErr_Fe_c = mean(aaErr_Fe_c);
%         MaaErr_Fe_t = mean(aaErr_Fe_t);
%         AngAxErr = [ MaaErr_Fe_c ; MaaErr_Fe_t ]
% 
%         stdaaErr_Fe_c = std(aaErr_Fe_c);
%         stdaaErr_Fe_t = std(aaErr_Fe_t);
%         stdAngAxErr = [ stdaaErr_Fe_c ; stdaaErr_Fe_t ]
%         
%         savefile = strcat( 'dat\', dir, parm_dir, '\Error', str_run_mode, '.mat' );
%         save( savefile, 'AngAxErr', 'stdAngAxErr' );                     
% 
          % ERROR PLOTS
        % Phi
%         h_aa_phi_err = figure;
%         plot( t', radtodeg(Aerr_Fe_c(  :, C_phi )), 'g', 'Linewidth', 2 )
%         hold on
%         plot( t', radtodeg(Aerr_Fe_t(  :, C_phi )), 'r', 'Linewidth', 2 )
%         legend( 'Fwd Euler Const', 'Fwd Euler Trap' )
%         title( 'ANG-AX:  Phi Error' )
%         xlabel( 'Time (s)')
%         ylabel( 'Error (deg)' )
%         grid on
%         savefile = strcat( 'dat\', dir, parm_dir, '\Phi_Err', str_run_mode, '.fig' );
%         saveas( h_aa_phi_err, savefile );        
% 
%         % Theta
%         h_aa_theta_err = figure;
%         plot( t', radtodeg(Aerr_Fe_c(  :, C_theta )), 'g', 'Linewidth', 2 )
%         hold on
%         plot( t', radtodeg(Aerr_Fe_t(  :, C_theta )), 'r', 'Linewidth', 2 )
%         legend( 'Fwd Euler Const', 'Fwd Euler Trap' );
%         title( 'ANGLE-AXIS:  Theta Error' )
%         xlabel( 'Time (s)')
%         ylabel( 'Error (deg)' )
%         grid on
%         savefile = strcat( 'dat\', dir, parm_dir, '\Theta_Err', str_run_mode, '.fig' );
%         saveas( h_aa_theta_err, savefile );        
% 
%         % Psi
%         h_aa_psi_err = figure;
%         plot( t', radtodeg(Aerr_Fe_c(  :, C_psi )), 'g' , 'Linewidth', 2 )
%         hold on
%         plot( t', radtodeg(Aerr_Fe_t(  :, C_psi )), 'r', 'Linewidth', 2 )
%         legend( 'Fwd Euler Const', 'Fwd Euler Trap' );
%         title( 'ANG-AX:  Psi Error' )
%         xlabel( 'Time (s)')
%         ylabel( 'Error (deg)' )
%         grid on
%         savefile = strcat( 'dat\', dir, parm_dir, '\Psi_Err', str_run_mode, '.fig' );
%         saveas( h_aa_psi_err, savefile ); 
%         
%         % DIFFERENCE SIGNALS FROM NOISE TESTS
%         if( idx ~= RUN_CLEAN )
%             % Phi
%             h_aa_phi_diff = figure;
%             plot( t', RK4_Err( :, C_phi), 'k' )
%             hold on
%             plot( t', Aerr_Fe_c(  :, C_phi ), 'g' )
%             plot( t', Aerr_Fe_t(  :, C_phi ), 'r' )
%             legend( 'True', 'Fwd Euler Const', 'Fwd Euler Avg')
%             title( 'ANG-AX - Phi Residual Signal' )
%             xlabel( 'Time (s)')
%             ylabel( 'Angle (deg)' )
%             grid on
%             savefile = strcat( 'dat\', dir, parm_dir, '\Phi_Diff', str_run_mode, '.fig' );
%             saveas( h_aa_phi_diff, savefile );        
% 
%             % Theta
%             h_aa_theta_diff = figure;
%             plot( t', RK4_Err( :, C_theta), 'k' )
%             hold on
%             plot( t', Aerr_Fe_c(  :, C_theta ), 'g' )
%             plot( t', Aerr_Fe_t(  :, C_theta ), 'r' )
%             legend( 'True', 'Fwd Euler Const', 'Fwd Euler Avg')
%             title( 'ANG-AX - Theta Residual Signal' )
%             xlabel( 'Time (s)')
%             ylabel( 'Angle (deg)' )
%             grid on
%             savefile = strcat( 'dat\', dir, parm_dir, '\Theta_Diff', str_run_mode, '.fig' );
%             saveas( h_aa_theta_diff, savefile ); 
% 
%             % Psi
%             h_aa_psi_diff = figure;
%             plot( t', RK4_Err( :, C_psi), 'k' )
%             hold on
%             plot( t', Aerr_Fe_c(  :, C_psi ), 'g' )
%             plot( t', Aerr_Fe_t(  :, C_psi ), 'r' )
%             legend( 'True', 'Fwd Euler Const', 'Fwd Euler Avg')
%             title( 'ANG-AX - Psi Residual Signal' )
%             xlabel( 'Time (s)')
%             ylabel( 'Angle (deg)' )
%             grid on
%             savefile = strcat( 'dat\', dir, parm_dir, '\Psi_Diff', str_run_mode, '.fig' );
%             saveas( h_aa_psi_diff, savefile ); 
%         end
% 
    end % aa_test plots
    
end % noise runs

if( DCM_TEST == 1 )
    disp( 'RMSE - NOISE RESPONSE')
    pmRMSE_rFec  = radtodeg(sqrt(mean( mRMSE_rFec )))
    pmRMSE_rFet  = radtodeg(sqrt(mean( mRMSE_rFet )))
    pmRMSE_rMecp = radtodeg(sqrt(mean( mRMSE_rMecp )))
    pmRMSE_rMecc = radtodeg(sqrt(mean( mRMSE_rMecc )))
    pmRMSE_rMet  = radtodeg(sqrt(mean( mRMSE_rMet )))
    pmRMSE_rMeb  = radtodeg(sqrt(mean( mRMSE_rMeb )))

    xlswrite( 'dcmRMSE_Noise2.xlsx', [pmRMSE_rFec; pmRMSE_rFet; pmRMSE_rMecp; ...
        pmRMSE_rMecc; pmRMSE_rMet ; pmRMSE_rMeb ] );
    
    
    if( idx ~= RUN_CLEAN )
        % PLOT RESIDUALS FOR ALL TESTS
        % Phi
        h_dcm_phi_diff = figure;
        plot( mT, mDiff_rk4( :, C_phi), 'k', 'linewidth',2);
        hold on
        plot( mT, mDiff_rFec(  :, C_phi ), 'b--' , 'linewidth',2);
        plot( mT, mDiff_rFet(  :, C_phi ), 'b-.' , 'linewidth',2);
        plot( mT, mDiff_rMecc( :, C_phi ), 'r--' , 'linewidth',2);
        plot( mT, mDiff_rMecp( :, C_phi ), 'r-.' , 'linewidth',2);
        plot( mT, mDiff_rMet(  :, C_phi ), 'g--' , 'linewidth',2);
        plot( mT, mDiff_rMeb(  :, C_phi ), 'g-.' , 'linewidth',2);
        legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
            'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
        title( 'DCM - Phi Residual Signal' )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Phi_Res', '.fig' );
        saveas( h_dcm_phi_diff, savefile );        

        % Theta
        h_dcm_theta_diff = figure;
        plot( mT, mDiff_rk4( :, C_theta), 'k' , 'linewidth',2);
        hold on
        plot( mT, mDiff_rFec(  :, C_theta ), 'b--' , 'linewidth',2);
        plot( mT, mDiff_rFet(  :, C_theta ), 'b-.' , 'linewidth',2);
        plot( mT, mDiff_rMecc( :, C_theta ), 'r--' , 'linewidth',2);
        plot( mT, mDiff_rMecp( :, C_theta ), 'r-.' , 'linewidth',2);
        plot( mT, mDiff_rMet(  :, C_theta ), 'g--' , 'linewidth',2);
        plot( mT, mDiff_rMeb(  :, C_theta ), 'g-.' , 'linewidth',2);
        legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
            'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
        title( 'DCM - Theta Residual Signal' )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Theta_Res', '.fig' );
        saveas( h_dcm_theta_diff, savefile ); 

        % Psi
        h_dcm_psi_diff = figure;
        plot( mT, mDiff_rk4( :, C_psi), 'k' , 'linewidth',2);
        hold on
        plot( mT, mDiff_rFec(  :, C_psi ), 'b--' , 'linewidth',2);
        plot( mT, mDiff_rFet(  :, C_psi ), 'b-.' , 'linewidth',2);
        plot( mT, mDiff_rMecc( :, C_psi ), 'r--' , 'linewidth',2);
        plot( mT, mDiff_rMecp( :, C_psi ), 'r-.' , 'linewidth',2);
        plot( mT, mDiff_rMet(  :, C_psi ), 'g--' , 'linewidth',2);
        plot( mT, mDiff_rMeb(  :, C_psi ), 'g-.' , 'linewidth',2);
        legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
            'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
        title( 'DCM - Psi Residual Signal' )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Psi_Res', '.fig' );
        saveas( h_dcm_psi_diff, savefile ); 

        max_rFEC = max( mDiff_rFec )
        max_rFET = max( mDiff_rFet )   
        max_rMECC = max( mDiff_rMecc )
        max_rMECP = max( mDiff_rMecp ) 
        max_rMET = max( mDiff_rMet )
        max_rMEB = max( mDiff_rMeb )     

        min_rFEC = min( mDiff_rFec )
        min_rFET = min( mDiff_rFet )
        min_rMECC = min( mDiff_rMecc )
        min_rMECP = min( mDiff_rMecp )
        min_rMET = min( mDiff_rMet )
        min_rMEB = min( mDiff_rMeb )    

        med_rFEC = median( mDiff_rFec )
        med_rFET = median( mDiff_rFet )
        med_rMECC = median( mDiff_rMecc )
        med_rMECP = median( mDiff_rMecp )
        med_rMET = median( mDiff_rMet )
        med_rMEB = median( mDiff_rMeb )    

        xlswrite( 'dcmResidualStats_Noise.xlsx', ...
            [ max_rFEC max_rFET max_rMECC max_rMECP max_rMET max_rMEB; ...
              min_rFEC min_rFET min_rMECC min_rMECP min_rMET min_rMEB; ...
              med_rFEC med_rFET med_rMECC med_rMECP med_rMET med_rMEB; ] );     

    end % noise testing
    
end % dcm_test
% 
if( EULER_TEST == 1 )
    
    disp( 'RMSE - NOISE RESPONSE')
    pmRMSE_eFec  = radtodeg(sqrt(mean( mRMSE_eFec ) ))
    pmRMSE_eFet  = radtodeg(sqrt(mean( mRMSE_eFet ) ))
    
    xlswrite( 'eulerRMSE_Noise2.xlsx', [pmRMSE_eFec; pmRMSE_eFet ] );    

    if( idx ~= RUN_CLEAN )      
        % PLOT RESIDUALS FOR ALL TESTS
        % phi
        h_euler_phi = figure;
        plot( mT, mDiff_rk4( :, C_phi ), 'k' , 'linewidth',2);
        hold on
        plot( mT, mDiff_eFec(  :, C_phi ), 'g' , 'linewidth',2); 
        plot( mT, mDiff_eFet(  :, C_phi ), 'r' , 'linewidth',2);
        legend( 'True', 'Fwd Euler Const', 'Fwd Euler Avg' )
        title( strcat( 'RESIDUAL: ', 'EULER - ', 'Phi (\phi) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Phi_Res', '.fig' );
        saveas( h_euler_phi, savefile );   

        % theta
        h_euler_theta = figure;
        plot( mT, mDiff_rk4( :, C_theta), 'k' , 'linewidth',2);
        hold on
        plot( mT, mDiff_eFec(  :, C_theta ), 'g' , 'linewidth',2); 
        plot( mT, mDiff_eFet(  :, C_theta ), 'r' , 'linewidth',2); 
        legend( 'True', 'Fwd Euler Const' , 'Fwd Euler Avg' )
        title( strcat('RESIDUAL: ',  'EULER - ', 'Theta (\theta) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Theta_Res', '.fig' );
        saveas( h_euler_theta, savefile );        

        % psi
        h_euler_psi = figure;
        plot( mT, mDiff_rk4( :, C_psi), 'k' , 'linewidth',2);
        hold on
        plot( mT, mDiff_eFec(  :, C_psi ), 'g' , 'linewidth',2);  
        plot( mT, mDiff_eFet(  :, C_psi ), 'r' , 'linewidth',2);
        legend( 'True', 'Fwd Euler Const' , 'Fwd Euler Avg' )
        title( strcat('RESIDUAL: ',  'EULER - ', 'Psi (\psi) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Psi_Res', '.fig' );
        saveas( h_euler_psi, savefile );  

        max_eFEC = max( mDiff_eFec )
        max_eFET = max( mDiff_eFet )    

        min_eFEC = min( mDiff_eFec )
        min_eFET = min( mDiff_eFet )

        med_eFEC = median( mDiff_eFec )
        med_eFET = median( mDiff_eFet )

        xlswrite( 'eulerResidualStats_Noise2.xlsx', [max_eFEC max_eFET; ...
            min_eFEC min_eFET; med_eFEC med_eFET ] ); 
    end % noise test

end  % euler_test
% 
if( QUAT_TEST == 1 )
    
    disp( 'RMSE - NOISE RESPONSE')
    pmRMSE_qFec  = radtodeg(sqrt(mean( mRMSE_qFec )))
    pmRMSE_qFet  = radtodeg(sqrt(mean( mRMSE_qFet )))
    pmRMSE_qMecp = radtodeg(sqrt(mean( mRMSE_qMecp )))
    pmRMSE_qMecc = radtodeg(sqrt(mean( mRMSE_qMecc )))
    pmRMSE_qMet  = radtodeg(sqrt(mean( mRMSE_qMet )))
    pmRMSE_qMeb  = radtodeg(sqrt(mean( mRMSE_qMeb )))

    xlswrite( 'quatRMSE_Noise2.xlsx', [pmRMSE_qFec; pmRMSE_qFet; pmRMSE_qMecp; ...
        pmRMSE_qMecc; pmRMSE_qMet ; pmRMSE_qMeb ] );
    
    if( idx ~= RUN_CLEAN )
        % PLOT RESIDUALS FOR ALL TESTS
        % Phi
        h_quat_phi_diff = figure;
        plot( mT, mDiff_rk4( :, C_phi), 'k', 'linewidth',2);
        hold on
        plot( mT, mDiff_qFec(  :, C_phi ), 'b--' , 'linewidth',2);
        plot( mT, mDiff_qFet(  :, C_phi ), 'b-.' , 'linewidth',2);
        plot( mT, mDiff_qMecc( :, C_phi ), 'r--' , 'linewidth',2);
        plot( mT, mDiff_qMecp( :, C_phi ), 'r-.' , 'linewidth',2);
        plot( mT, mDiff_qMet(  :, C_phi ), 'g--' , 'linewidth',2);
        plot( mT, mDiff_qMeb(  :, C_phi ), 'g-.' , 'linewidth',2);
        legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
            'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
        title( 'QUAT - Phi Residual Signal' )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Phi_Res', '.fig' );
        saveas( h_quat_phi_diff, savefile );        

        % Theta
        h_quat_theta_diff = figure;
        plot( mT, mDiff_rk4( :, C_theta), 'k', 'linewidth',2);
        hold on
        plot( mT, mDiff_qFec(  :, C_theta ), 'b--' , 'linewidth',2);
        plot( mT, mDiff_qFet(  :, C_theta ), 'b-.' , 'linewidth',2);
        plot( mT, mDiff_qMecc( :, C_theta ), 'r--' , 'linewidth',2);
        plot( mT, mDiff_qMecp( :, C_theta ), 'r-.' , 'linewidth',2);
        plot( mT, mDiff_qMet(  :, C_theta ), 'g--' , 'linewidth',2);
        plot( mT, mDiff_qMeb(  :, C_theta ), 'g-.', 'linewidth',2);
        legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
            'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
        title( 'QUAT - Theta Residual Signal' )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Theta_Res', '.fig' );
        saveas( h_quat_theta_diff, savefile ); 

        % Psi
        h_quat_psi_diff = figure;
        plot( mT, mDiff_rk4( :, C_psi), 'k', 'linewidth',2);
        hold on
        plot( mT, mDiff_qFec(  :, C_psi ), 'b--', 'linewidth',2);
        plot( mT, mDiff_qFet(  :, C_psi ), 'b-.' , 'linewidth',2);
        plot( mT, mDiff_qMecc( :, C_psi ), 'r--' , 'linewidth',2);
        plot( mT, mDiff_qMecp( :, C_psi ), 'r-.' , 'linewidth',2);
        plot( mT, mDiff_qMet(  :, C_psi ), 'g--' , 'linewidth',2);
        plot( mT, mDiff_qMeb(  :, C_psi ), 'g-.', 'linewidth',2);
        legend( 'True', 'Fwd Euler Const', 'Fwd Euler Trap', 'MExp Const Curr', ...
            'MExp Const Prev', 'MExp Trap', 'MExp OmBar')
        title( 'QUAT - Psi Residual Signal' )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Psi_Res', '.fig' );
        saveas( h_quat_psi_diff, savefile ); 

        max_qFEC  = max( mDiff_qFec )
        max_qFET  = max( mDiff_qFet )   
        max_qMECC = max( mDiff_qMecc )
        max_qMECP = max( mDiff_qMecp ) 
        max_qMET  = max( mDiff_qMet )
        max_qMEB  = max( mDiff_qMeb )     

        min_qFEC  = min( mDiff_qFec )
        min_qFET  = min( mDiff_qFet )
        min_qMECC = min( mDiff_qMecc )
        min_qMECP = min( mDiff_qMecp )
        min_qMET  = min( mDiff_qMet )
        min_qMEB  = min( mDiff_qMeb )    

        med_qFEC  = median( mDiff_qFec )
        med_qFET  = median( mDiff_qFet )
        med_qMECC = median( mDiff_qMecc )
        med_qMECP = median( mDiff_qMecp )
        med_qMET  = median( mDiff_qMet )
        med_qMEB  = median( mDiff_qMeb )    

        xlswrite( 'quatResidualStats_Noise2.xlsx', ...
            [ max_qFEC max_qFET max_qMECC max_qMECP max_qMET max_qMEB; ...
              min_qFEC min_qFET min_qMECC min_qMECP min_qMET min_qMEB; ...
              med_qFEC med_qFET med_qMECC med_qMECP med_qMET med_qMEB; ] );     
    end % noise test

end  % quat test
% 
if( AA_TEST == 1 )
    disp( 'RMSE - NOISE RESPONSE')
    pmRMSE_aFec  = radtodeg(sqrt(mean( mRMSE_aFec ) ))
    pmRMSE_aFet  = radtodeg(sqrt(mean( mRMSE_aFet ) ))
    
    xlswrite( 'angaxRMSE_Noise2.xlsx', [pmRMSE_aFec; pmRMSE_aFet ] ); 
    
    if( idx ~= RUN_CLEAN )
        % PLOT RESIDUALS FOR ALL TESTS
        % phi
        h_aa_phi = figure;
        plot( mT, mDiff_rk4( :, C_phi ), 'k' , 'linewidth',2);
        hold on
        plot( mT, mDiff_aFec(  :, C_phi ), 'g' , 'linewidth',2);  
        plot( mT, mDiff_aFet(  :, C_phi ), 'r' , 'linewidth',2); 
        legend( 'True', 'Fwd Euler Const', 'Fwd Euler Avg' )
        title( strcat( 'RESIDUAL: ', 'Angle-Axis - ', 'Phi (\phi) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Phi_Res', '.fig' );
        saveas( h_aa_phi, savefile );   

        % theta
        h_aa_theta = figure;
        plot( mT, mDiff_rk4( :, C_theta), 'k' , 'linewidth',2);
        hold on
        plot( mT, mDiff_aFec(  :, C_theta ), 'g' , 'linewidth',2);  
        plot( mT, mDiff_aFet(  :, C_theta ), 'r' , 'linewidth',2); 
        legend( 'True', 'Fwd Euler Const' , 'Fwd Euler Avg' )
        title( strcat('RESIDUAL: ',  'Angle-Axis - ', 'Theta (\theta) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Theta_Res', '.fig' );
        saveas( h_aa_theta, savefile );        

        % psi
        h_aa_psi = figure;
        plot( mT, mDiff_rk4( :, C_psi), 'k' , 'linewidth',2);
        hold on
        plot( mT, mDiff_aFec(  :, C_psi ), 'g' , 'linewidth',2); 
        plot( mT, mDiff_aFet(  :, C_psi ), 'r' , 'linewidth',2); 
        legend( 'True', 'Fwd Euler Const' , 'Fwd Euler Avg' )
        title( strcat('RESIDUAL: ',  'Angle-Axis - ', 'Psi (\psi) - ', mode_str) )
        xlabel( 'Time (s)')
        ylabel( 'Angle (deg)' )
        grid on
        savefile = strcat( 'dat\', dir, parm_dir, '\Psi_Res', '.fig' );
        saveas( h_aa_psi, savefile );  

        max_aFEC = max( mDiff_aFec )
        max_aFET = max( mDiff_aFet )    

        min_aFEC = min( mDiff_aFec )
        min_aFET = min( mDiff_aFet )

        med_aFEC = median( mDiff_aFec )
        med_aFET = median( mDiff_aFet )

        xlswrite( 'aaResidualStats_Noise2.xlsx', [max_aFEC max_aFET; ...
            min_aFEC min_aFET; med_aFEC med_aFET ] ); 
    end % noise test

end % aa_test

if( NOISE_ON == 1 )
    RMSE_rk4 = radtodeg(sqrt( mean( diffE_q )))
    
    max_rk4 = (max( mDiff_rk4 ))   
    min_rk4 = (min( mDiff_rk4 ))
    med_rk4 = (median( mDiff_rk4 ))
    xlswrite( 'rk4RMSE2.xlsx', [ RMSE_rk4; max_rk4; min_rk4; med_rk4 ] ); 
end
    