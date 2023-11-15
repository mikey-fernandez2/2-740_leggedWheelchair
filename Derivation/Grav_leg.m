function Grav_Joint_Sp = Grav_leg(in1,in2)
%Grav_leg
%    Grav_Joint_Sp = Grav_leg(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    15-Nov-2023 11:00:15

b = in2(17,:);
g = in2(30,:);
lU = in2(28,:);
l_AC = in2(15,:);
l_A_m3 = in2(20,:);
l_B_m2 = in2(19,:);
l_C_m4 = in2(21,:);
l_OA = in2(13,:);
l_OB = in2(14,:);
l_O_m1 = in2(18,:);
l_cb = in2(22,:);
m1 = in2(1,:);
m2 = in2(2,:);
m3 = in2(3,:);
m4 = in2(4,:);
mU = in2(26,:);
ma = in2(5,:);
mb = in2(6,:);
phiU = in2(29,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
t2 = cos(th5);
t3 = l_AC.*m4;
t4 = l_A_m3.*m3;
t5 = l_B_m2.*m2;
t6 = th3+th5;
t9 = -th5;
t7 = sin(t6);
t8 = t6+th4;
t11 = t9+th1;
t22 = t3+t4+t5;
t10 = sin(t8);
t12 = sin(t11);
t13 = t11+th2;
t14 = l_C_m4.*m4.*t7;
t15 = l_OA.*m3.*t7;
t16 = l_OA.*m4.*t7;
t17 = l_O_m1.*m1.*t7;
t18 = t3.*t10;
t19 = t4.*t10;
t20 = t5.*t10;
t21 = sin(t13);
t23 = l_C_m4.*m4.*t12;
t24 = l_OA.*m3.*t12;
t25 = l_OA.*m4.*t12;
t26 = l_O_m1.*m1.*t12;
t27 = l_OB.*m2.*t12.*2.0;
t28 = t3.*t21;
t29 = t4.*t21;
t30 = t5.*t21;
Grav_Joint_Sp = [g.*(t23+t24+t25+t26+t27+t28+t29+t30);g.*t21.*t22;g.*(t14+t15+t16+t17+t18+t19+t20);g.*t10.*t22;g.*(t14+t15+t16+t17+t18+t19+t20-t23-t24-t25-t26-t27-t28-t29-t30+b.*m1.*t2.*2.0+b.*m2.*t2.*2.0+b.*m3.*t2.*2.0+b.*m4.*t2.*2.0+l_cb.*mb.*t2+lU.*mU.*cos(phiU+th5));0.0;g.*(m1.*2.0+m2.*2.0+m3.*2.0+m4.*2.0+mU+ma+mb);0.0];
