%% Linearization

Q = [100 0 0 0 0 0 0 0;
     0 100 0 0 0 0 0 0;
     0 0 100 0 0 0 0 0;
     0 0 0 100 0 0 0 0;
     0 0 0 0 100 0 0 0;
     0 0 0 0 0 100 0 0;
     0 0 0 0 0 0 100 0;
     0 0 0 0 0 0 0 100];
 
R = eye(4);

[A,B,C,D] = update_state_matrices([0 0 0 0 0 0 0 0]);

sys = ss(A,B,C,D);

dsys = c2d(sys, 0.002);

[K,S,E] = dlqr(A,B,Q,R);

Kr = pinv(dsys.d-(dsys.c-dsys.d*K)*inv(dsys.a-dsys.b*K)*dsys.b);