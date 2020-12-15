%PROJECT CODE 1
clc
clear 
Ra = 7.31; % Armature Winding Resistance in ohms
La = 8.3*10^-4; % Armature Winding Inductance in H
Kb = 22.7; % Back emf constant in rad/Vs
Kt = 0.044; % Motor Torque Constant in Nm/A
Jm = 1.05*10^-6; % Moment of Inertia of the Rotor in kg*m^2
Bm = 2.0*10^-6; % Damping Coeffecient in Nms
A = [-Ra/La -Kt/La; Kt/Jm -Bm/Jm];
B = [1/La; 0];
Ts = 0;
C = [0  1];
D = 0;
DC_Motor = ss(A,B,C,D,Ts,...
   'InputName','V','OutputName','Y','StateName',{'i','theta_dot'}) % State Space Model

Amplifier = tf(24, [1/2500 1]) % Voltage Amplifier with Steady state Voltage Gain and Band Width
Plant = DC_Motor * Amplifier % Overall Plant Model
Plant.StateName{3} = 'x3'
[V,E] = eig(A) % Eigen Vectors and Eigen Values of A
Poles = pole(DC_Motor) % Stability Analysis.

step(Plant) % Step Response for a unit step voltage reprsenting the rise time, the settling time and the steady state

Controlability  = ctrb(DC_Motor) % Controlability Analysis
Rank_C = rank(Controlability) % Linearly Independent Columns to Prove Controlability

Observability = obsv(DC_Motor) % Observability Analysis
Rank_O = rank(Observability) % Linearly Independent Rows to prove Observability

% Full State feedback Control
Plant.C = eye(3) % Modifying C Matrix into Identity Matrix
Plant.StateName{3} = 'x3'
Plant.OutputName = {'i','theta_dot','x3'}

%Pole Placement
%K = place(Plant.A, Plant.B, [ -7 + 5.25j, -7 - 5.25j, -21]) % State Space Matrices A and B and Desired Pole Position
 
% Closed Loop State Space Model using Feedback Command
%C1 = feedback(Plant, K)

% step(C1(1))

% Kalman Filter
Q = 10^-3; % Process Covariance Matrix
R = 10^-4; % Measurement Covariance Matrix
[kalmanf,L,P] = kalman(Plant,Q,R); 
kalmanf = kalmanf(1,:) % Output Estimate
sys = parallel(Plant,kalmanf,1,1,[],[]); % Connecting Plant Model and Kalman Filter in Parallel

SimModel = feedback(sys,1,2,3);
SimModel = SimModel([1 2],[1 2 3]);
SimModel.InputName
SimModel.OutputName

%this is the part i am trying to comprehend to plot 
t = [0:100]'
u = sin(t/6)

rng(10,'twister');
w = sqrt(Q)*randn(length(t),1)
v = sqrt(R)*randn(length(t),1)

