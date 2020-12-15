% PROJECT CODE 2
clear 
clc
V = 24;
Bm = 2*(10^-6);
L = 8.3*(10^-4);
R = 7.31;
J = 1.05*(10^-6);
Kt = 0.044;

A = [-R/L -Kt/L; Kt/J -Bm/J]
B = [1/L;0] 
C = [0 1];
D =0;
p=ss(A,B,C,D);
p_order = order(p)
p_rank = rank(ctrb(A,B))
s = tf('s');
p_motor = Kt/((J*s+Bm)*(L*s+R)+Kt^2);
zpk(p_motor)
% To calculate the state transition matrix for A
pd=c2d(p,0.001)
% Process noise covariance
nQ = 1e-3;
% Measurement noise covariance
nR = 1e-4;
Ts= 0.001;
