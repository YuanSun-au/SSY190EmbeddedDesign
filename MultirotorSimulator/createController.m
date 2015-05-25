function controller = createController()
Q = [100000000 0 0 0 0 0 0 0;
     0 100000000 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0;
     0 0 0 2000 0 0 0 0;
     0 0 0 0 2000 0 0 0;
     0 0 0 0 0 100000000 0 0;
     0 0 0 0 0 0 30 0;
     0 0 0 0 0 0 0 1];


% Q = [10000000 0 0 0 0 0 0 0;
%      0 10000000 0 0 0 0 0 0;
%      0 0 1 0 0 0 0 0;
%      0 0 0 2000 0 0 0 0;
%      0 0 0 0 2000 0 0 0;
%      0 0 0 0 0 100000000 0 0;
%      0 0 0 0 0 0 30 0;
%      0 0 0 0 0 0 0 1];
 
R = eye(4);

[A,B,C,D] = update_state_matrices([0 0 0 0 0 0 0 0]);

sys = ss(A,B,C,D);

dsys = c2d(sys, 0.002);

[K,S,E] = dlqr(dsys.a,dsys.b,Q,R);

Kr = pinv(dsys.d-(dsys.c-dsys.d*K)*inv(dsys.a-dsys.b*K)*dsys.b);

controller = struct( 'Kmat',K,...
                     'Krmat', Kr);
K

end