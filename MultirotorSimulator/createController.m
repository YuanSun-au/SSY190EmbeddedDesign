function controller = createController(mr)
%usage: createController(createMultirotor())
clc
% Q = [100000000 0 0 0 0 0 0 0;
%      0 100000000 0 0 0 0 0 0;
%      0 0 1 0 0 0 0 0;
%      0 0 0 2000000 0 0 0 0;
%      0 0 0 0 2000000 0 0 0;
%      0 0 0 0 0 100000000 0 0;
%      0 0 0 0 0 0 1 0;
%      0 0 0 0 0 0 0 1];
Propi =1.5e3;
Dropi =5;
Dya = 5e1;
 Q = diag([Propi,Propi, 0,Dropi,Dropi, Dya, 1, 0 ]);


% Q = [10000000 0 0 0 0 0 0 0;
%      0 10000000 0 0 0 0 0 0;
%      0 0 1 0 0 0 0 0;
%      0 0 0 2000 0 0 0 0;
%      0 0 0 0 2000 0 0 0;
%      0 0 0 0 0 100000000 0 0;
%      0 0 0 0 0 0 30 0;
%      0 0 0 0 0 0 0 1];
 
R = eye(4);

[A,B,C,D] = update_state_matrices(mr);

sys = ss(A,B,C,D);

dsys = c2d(sys, 0.002);

[K,S,E] = dlqr(dsys.a,dsys.b,Q,R);

Kr = pinv(dsys.d-(dsys.c-dsys.d*K)*inv(dsys.a-dsys.b*K)*dsys.b);

controller = struct( 'Kmat',K,...
                     'Krmat', Kr);
K
Kseeked = [0.2, 0.2, 0,0.03,0.03,0.2,0.4940,0.5371]


end