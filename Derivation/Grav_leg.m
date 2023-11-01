function Grav_Joint_Sp = Grav_leg(in1,in2)
%Grav_leg
%    Grav_Joint_Sp = Grav_leg(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    31-Oct-2023 21:50:52

b = in2(16,:);
g = in2(23,:);
l1 = in2(14,:);
lU = in2(1,:);
l_c1 = in2(17,:);
l_c2 = in2(18,:);
l_cb = in2(19,:);
m1 = in2(5,:);
m2 = in2(6,:);
mU = in2(3,:);
ma = in2(7,:);
mb = in2(8,:);
phiU = in2(2,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
t2 = cos(th5);
t3 = th1+th5;
t4 = th3+th5;
t5 = sin(t3);
t6 = sin(t4);
t7 = t3+th2;
t8 = t4+th4;
t9 = sin(t7);
t10 = sin(t8);
t11 = l1.*m2.*t5;
t12 = l1.*m2.*t6;
t13 = l_c1.*m1.*t5;
t14 = l_c1.*m1.*t6;
t15 = l_c2.*m2.*t9;
t16 = l_c2.*m2.*t10;
Grav_Joint_Sp = [g.*(t11+t13+t15);g.*t15;g.*(t12+t14+t16);g.*t16;g.*(t11+t12+t13+t14+t15+t16+b.*m1.*t2.*2.0+b.*m2.*t2.*2.0+l_cb.*mb.*t2+lU.*mU.*cos(phiU+th5));0.0;g.*(m1.*2.0+m2.*2.0+mU+ma+mb);0.0];
