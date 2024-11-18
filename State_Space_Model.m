%Vehicle model but in State-Space
%state vector X = [x y z x' y' z' phi theta psi phi' theta' psi']' and
%control input u = [f_l T_phi T_theta T_psi]' ==> A is 12x12 and B is 12X4
%u is dependent on [w1^2 w2^2 w3^2 w4^2]'
g = 9.81;
A = [
    0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 -g 0 0 0 0;
    0 0 0 0 0 0 g 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 -gamma/ITzz];
B = [0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     1/m 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 1/ITxx 0 0;
     0 0 1/ITyy 0;
     0 0 0 1/ITzz];
%B matrix for rotor speed inputs
B_res = B * [kf kf kf kf; 0 l*kf 0 -l*kf; -l*kf 0 l*kf 0; kt*kf -kt*kf kt*kf -kt*kf];
C = [0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0]; %For angular position(w.r.t body frame) output(don't care about yaw)
D = 0;
vehicle = ss(A, B, C, D);
%One propeller loss
