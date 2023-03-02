b = 0.000135;
J = 0.008586328125;
kt= 0.7614;
R = 1.1;
L = 1.55e-3;
kb = 0.178;

A = [-R/L -kb/L; kt/J -b/J  ];
B = [1.0/L;0];
C = [0,1];
D = 0;
sys = ss(A,B,C,D)

step(sys)

Ts = 0.0001;

sys_tf = tf(sys)
sys_d = c2d( sys, Ts, 'foh' )sysd
