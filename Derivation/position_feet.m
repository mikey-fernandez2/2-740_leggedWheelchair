function rFeet = position_feet(in1,in2)
%POSITION_FEET
%    rFeet = POSITION_FEET(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    06-Nov-2023 17:21:16

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
t4 = th1+th5;
t5 = th3+th5;
t6 = b.*t2;
t7 = b.*t3;
t8 = cos(t4);
t9 = cos(t5);
t10 = sin(t4);
t11 = sin(t5);
t12 = t4+th2;
t13 = t5+th4;
rFeet = reshape([t6+x-l_DE.*t10-l_OB.*t10-l_AC.*sin(t12),t7+y-l_DE.*t8-l_OB.*t8-l_AC.*cos(t12),t6+x+l_DE.*t11+l_OB.*t11+l_AC.*sin(t13),t7+y-l_DE.*t9-l_OB.*t9-l_AC.*cos(t13)],[2,2]);
