syms q1 q2 q3 q4 q5 q6 q7
%alpha  d   a   theta
num_joint = 7;
dh_table = [ pi/2,  0.36,    0,  q1;
            -pi/2,     0,    0,  q2;
            -pi/2,  0.42,    0,  q3;
             pi/2,     0,    0,  q4;
             pi/2,  0.40,    0,  q5;
            -pi/2,     0,    0,  q6;
                0,  0.21,    0,  q7];
         
joint_limits = [      -pi,        pi;
                  -2*pi/3,    2*pi/3;
                      -pi,        pi;
                  -2*pi/3,    2*pi/3;
                      -pi,        pi;
                -13*pi/18,  13*pi/18;   
                      -pi,        pi];
                
                