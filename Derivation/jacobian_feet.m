function J = jacobian_feet(in1,in2)
%JACOBIAN_FEET
%    J = JACOBIAN_FEET(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    06-Nov-2023 17:21:17

b = in2(17,:);
l_AC = in2(15,:);
l_DE = in2(16,:);
l_OB = in2(14,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
t2 = sin(th5);
t3 = th1+th5;
t4 = th3+th5;
t5 = b.*t2;
t6 = cos(t3);
t7 = cos(t4);
t8 = sin(t3);
t9 = t3+th2;
t10 = t4+th4;
t11 = cos(t9);
t12 = cos(t10);
t13 = sin(t9);
t14 = l_DE.*t6;
t15 = l_DE.*t7;
t16 = l_OB.*t6;
t17 = l_OB.*t7;
t18 = l_DE.*t8;
t19 = l_OB.*t8;
t20 = -t5;
t21 = l_AC.*t11;
t22 = l_AC.*t12;
t23 = l_AC.*t13;
t24 = -t14;
t25 = -t16;
t26 = -t21;
J = reshape([t24+t25+t26,t18+t19+t23,0.0,0.0,t26,t23,0.0,0.0,0.0,0.0,0.0,t15+t17+t22,0.0,0.0,0.0,t22,t20+t24+t25+t26,t18+t19+t23+b.*cos(th5),0.0,t15+t17+t20+t22,1.0,0.0,0.0,1.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0],[4,8]);
