function userAccel = acceleration_user(in1,in2,in3)
%ACCELERATION_USER
%    userAccel = ACCELERATION_USER(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    01-Dec-2023 15:01:41

ddth5 = in2(13,:);
ddx = in2(14,:);
ddy = in2(15,:);
dth5 = in1(13,:);
lU = in3(28,:);
phiU = in3(29,:);
th5 = in1(5,:);
t2 = phiU+th5;
t3 = dth5.^2;
t4 = cos(t2);
t5 = sin(t2);
userAccel = [ddx-ddth5.*lU.*t5-lU.*t3.*t4;ddy+ddth5.*lU.*t4-lU.*t3.*t5;ddth5];
