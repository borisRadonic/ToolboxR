b = 0.000135;
J = 0.08586328125;
kt= 0.7614;
R = 1.1;
L = 1.55e-3;
kb = 0.178;

Kp1 = 3.5;
Ki1 = 33.2;
Kaw1 = Ki1;

Kp2 = 2.00174495936295;
Ki2 = 30.1;
Kaw2 = Ki2;

A = [-R/L -kb/L; kt/J -b/J  ];
B = [120.0/L;0];
C = [0,1];
D = 0;
sys = ss(A,B,C,D)

step(sys)

Ts = 0.0001;

sys_tf = tf(sys)
sys_d = c2d( sys, Ts, 'foh' )
