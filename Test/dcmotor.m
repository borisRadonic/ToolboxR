b = 0.000135;
J = 0.008586328125;
kt= 0.7614;
R = 1.1;
L = 1.55e-3;
kb = 0.178;

Kp1 = 2.0;
Ki1 = 30.2;
Kaw1 = 1.0;

Kp2 = 2.00174495936295;
Ki2 = 1904.96918720126;
Kaw2 = 1.0;

A = [-R/L -kb/L; kt/J -b/J  ];
B = [1.0/L;0];
C = [0,1];
D = 0;
sys = ss(A,B,C,D)

step(sys)

Ts = 0.0001;

sys_tf = tf(sys)
sys_d = c2d( sys, Ts, 'foh' )
