function [ y ] = BarOrthog( D )
%BARORTHOG Testbed for the dual iterative DCM orthogonalization algorithm.
% D = input 3x3 matrix, the attitude DCM, assumed to have drifted somewhat 
% out of orthonormality
% Here we are using eq'n (18), the "recursive" (here we use an iterative) 
% version of I. Bar-Itzhack's algorithm from the paper 
% "Strapdown Matrix Orthogonalization: the Dual Iterative Algorithm", 1976, 
% IEEE Transactions on Aerospace and Electronic Systems.

% Init
y = D;

% basic setup is to repeat the sequence until the sequence
% converges to an acceptable error rate, E_TOL
% core algorithm:  X_{n+1} = (1/2) * (X_n^T)^-1 + (1/2)X_n
E_TOL = 1e-12;
E_curr = 100; % initial value

I3 = eye(3);
while ( E_curr > E_TOL )
    y = (1/2) * pinv(D') + (1/2) * D;
    
    E_curr = norm( I3 - D * D' );
    
    D = y;  % 'lastvalue = current value' idiom:  update DCM 
        
end % while

end % function


