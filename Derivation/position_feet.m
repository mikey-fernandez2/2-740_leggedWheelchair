function rFeet = position_feet(in1,in2)
%POSITION_FEET
%    rFeet = POSITION_FEET(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    14-Nov-2023 15:06:22

b = in2(17,:);
l_AC = in2(15,:);
l_DE = in2(16,:);
l_OB = in2(14,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
x = in1(6,:);
y = in1(7,:);
t2 = cos(th5);
t3 = sin(th5);
t4 = th3+th5;
t10 = -th5;
t5 = b.*t2;
t6 = b.*t3;
t7 = cos(t4);
t8 = sin(t4);
t9 = t4+th4;
t11 = t10+th1;
t12 = cos(t11);
t13 = sin(t11);
t14 = t11+th2;
rFeet = reshape([t5+x-l_DE.*t13-l_OB.*t13-l_AC.*sin(t14),t6+y-l_DE.*t12-l_OB.*t12-l_AC.*cos(t14),t5+x+l_DE.*t8+l_OB.*t8+l_AC.*sin(t9),t6+y-l_DE.*t7-l_OB.*t7-l_AC.*cos(t9)],[2,2]);
