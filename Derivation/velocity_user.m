function drU = velocity_user(in1,in2)
%VELOCITY_USER
%    drU = VELOCITY_USER(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    30-Nov-2023 15:15:38

dth5 = in1(13,:);
dx = in1(14,:);
dy = in1(15,:);
lU = in2(28,:);
phiU = in2(29,:);
th5 = in1(5,:);
t2 = phiU+th5;
drU = [dx-dth5.*lU.*sin(t2);dy+dth5.*lU.*cos(t2);0.0];
