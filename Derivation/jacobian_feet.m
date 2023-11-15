function J = jacobian_feet(in1,in2)
%JACOBIAN_FEET
%    J = JACOBIAN_FEET(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    15-Nov-2023 13:55:15

b = in2(17,:);
l_AC = in2(15,:);
l_DE = in2(16,:);
l_OB = in2(14,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
t2 = cos(th5);
t3 = sin(th5);
t4 = th3+th5;
t10 = -th5;
t5 = b.*t2;
t6 = b.*t3;
t7 = cos(t4);
t8 = sin(t4);
t9 = t4+th4;
t17 = t10+th1;
t11 = cos(t9);
t12 = sin(t9);
t13 = l_DE.*t7;
t14 = l_OB.*t7;
t15 = l_DE.*t8;
t16 = l_OB.*t8;
t18 = -t6;
t19 = cos(t17);
t20 = sin(t17);
t21 = t17+th2;
t22 = l_AC.*t11;
t23 = l_AC.*t12;
t24 = cos(t21);
t25 = sin(t21);
t26 = l_DE.*t19;
t27 = l_OB.*t19;
t28 = l_DE.*t20;
t29 = l_OB.*t20;
t30 = l_AC.*t24;
t31 = l_AC.*t25;
t32 = -t30;
J = reshape([-t26-t27+t32,t28+t29+t31,0.0,0.0,0.0,0.0,0.0,0.0,t32,t31,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t13+t14+t22,t15+t16+t23,0.0,0.0,0.0,0.0,0.0,0.0,t22,t23,0.0,0.0,0.0,0.0,t18+t26+t27+t30,t5-t28-t29-t31,t13+t14+t18+t22,t5+t15+t16+t23,0.0,0.0,t18,t5,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[8,8]);
