function rU = position_user(in1,in2)
%POSITION_USER
%    rU = POSITION_USER(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    15-Nov-2023 11:00:13

lU = in2(28,:);
phiU = in2(29,:);
th5 = in1(5,:);
x = in1(6,:);
y = in1(7,:);
t2 = phiU+th5;
rU = [x+lU.*cos(t2);y+lU.*sin(t2);0.0];
