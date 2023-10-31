function Corr_Joint_Sp = Corr_leg(in1,in2)
%Corr_leg
%    Corr_Joint_Sp = Corr_leg(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    30-Oct-2023 20:51:58

b = in2(16,:);
dth1 = in1(9,:);
dth2 = in1(10,:);
dth3 = in1(11,:);
dth4 = in1(12,:);
dth5 = in1(13,:);
l1 = in2(14,:);
lU = in2(1,:);
l_c1 = in2(17,:);
l_c2 = in2(18,:);
l_cb = in2(19,:);
m1 = in2(5,:);
m2 = in2(6,:);
mU = in2(3,:);
mb = in2(8,:);
phiU = in2(2,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
t2 = cos(th1);
t3 = cos(th3);
t4 = cos(th5);
t5 = sin(th2);
t6 = sin(th4);
t7 = sin(th5);
t8 = phiU+th5;
t9 = th1+th2;
t10 = th1+th5;
t11 = th3+th4;
t12 = th3+th5;
t13 = dth1.^2;
t14 = dth2.^2;
t15 = dth3.^2;
t16 = dth4.^2;
t17 = dth5.^2;
t18 = cos(t9);
t19 = cos(t10);
t20 = cos(t11);
t21 = cos(t12);
t22 = sin(t10);
t23 = sin(t12);
t24 = t9+th5;
t25 = t11+th5;
t30 = l1.*l_c2.*m2.*t5.*t14;
t31 = l1.*l_c2.*m2.*t6.*t16;
t32 = dth1.*dth2.*l1.*l_c2.*m2.*t5.*2.0;
t33 = dth2.*dth5.*l1.*l_c2.*m2.*t5.*2.0;
t34 = dth3.*dth4.*l1.*l_c2.*m2.*t6.*2.0;
t35 = dth4.*dth5.*l1.*l_c2.*m2.*t6.*2.0;
t26 = cos(t24);
t27 = cos(t25);
t28 = sin(t24);
t29 = sin(t25);
t36 = -t32;
t37 = -t33;
t38 = -t34;
t39 = -t35;
t40 = -t30;
t41 = -t31;
et1 = b.*m1.*t4.*t17.*-2.0-b.*m2.*t4.*t17.*2.0-l1.*m2.*t13.*t22-l1.*m2.*t15.*t23-l1.*m2.*t17.*t22-l1.*m2.*t17.*t23-l_c1.*m1.*t13.*t22-l_c1.*m1.*t15.*t23-l_c1.*m1.*t17.*t22-l_c1.*m1.*t17.*t23-l_c2.*m2.*t13.*t28-l_c2.*m2.*t14.*t28-l_c2.*m2.*t15.*t29-l_c2.*m2.*t16.*t29-l_c2.*m2.*t17.*t28-l_c2.*m2.*t17.*t29-l_cb.*mb.*t4.*t17-lU.*mU.*t17.*cos(t8)-dth1.*dth5.*l1.*m2.*t22.*2.0-dth3.*dth5.*l1.*m2.*t23.*2.0-dth1.*dth5.*l_c1.*m1.*t22.*2.0-dth3.*dth5.*l_c1.*m1.*t23.*2.0-dth1.*dth2.*l_c2.*m2.*t28.*2.0-dth1.*dth5.*l_c2.*m2.*t28.*2.0-dth2.*dth5.*l_c2.*m2.*t28.*2.0-dth3.*dth4.*l_c2.*m2.*t29.*2.0-dth3.*dth5.*l_c2.*m2.*t29.*2.0;
et2 = dth4.*dth5.*l_c2.*m2.*t29.*-2.0;
mt1 = [t36+t37+t40-b.*l1.*m2.*t2.*t17-b.*l_c1.*m1.*t2.*t17-b.*l_c2.*m2.*t17.*t18;l_c2.*m2.*(-b.*t17.*t18+l1.*t5.*t13+l1.*t5.*t17+dth1.*dth5.*l1.*t5.*2.0);t38+t39+t41-b.*l1.*m2.*t3.*t17-b.*l_c1.*m1.*t3.*t17-b.*l_c2.*m2.*t17.*t20;l_c2.*m2.*(-b.*t17.*t20+l1.*t6.*t15+l1.*t6.*t17+dth3.*dth5.*l1.*t6.*2.0)];
mt2 = [t36+t37+t38+t39+t40+t41+b.*l1.*m2.*t2.*t13+b.*l1.*m2.*t3.*t15+b.*l_c1.*m1.*t2.*t13+b.*l_c1.*m1.*t3.*t15+b.*l_c2.*m2.*t13.*t18+b.*l_c2.*m2.*t14.*t18+b.*l_c2.*m2.*t15.*t20+b.*l_c2.*m2.*t16.*t20+b.*dth1.*dth5.*l1.*m2.*t2.*2.0+b.*dth3.*dth5.*l1.*m2.*t3.*2.0+b.*dth1.*dth5.*l_c1.*m1.*t2.*2.0+b.*dth3.*dth5.*l_c1.*m1.*t3.*2.0+b.*dth1.*dth2.*l_c2.*m2.*t18.*2.0+b.*dth1.*dth5.*l_c2.*m2.*t18.*2.0+b.*dth2.*dth5.*l_c2.*m2.*t18.*2.0+b.*dth3.*dth4.*l_c2.*m2.*t20.*2.0+b.*dth3.*dth5.*l_c2.*m2.*t20.*2.0+b.*dth4.*dth5.*l_c2.*m2.*t20.*2.0;et1+et2];
mt3 = [b.*m1.*t7.*t17.*-2.0-b.*m2.*t7.*t17.*2.0+l1.*m2.*t13.*t19+l1.*m2.*t15.*t21+l1.*m2.*t17.*t19+l1.*m2.*t17.*t21+l_c1.*m1.*t13.*t19+l_c1.*m1.*t15.*t21+l_c1.*m1.*t17.*t19+l_c1.*m1.*t17.*t21+l_c2.*m2.*t13.*t26+l_c2.*m2.*t14.*t26+l_c2.*m2.*t15.*t27+l_c2.*m2.*t16.*t27+l_c2.*m2.*t17.*t26+l_c2.*m2.*t17.*t27-l_cb.*mb.*t7.*t17-lU.*mU.*t17.*sin(t8)+dth1.*dth5.*l1.*m2.*t19.*2.0+dth3.*dth5.*l1.*m2.*t21.*2.0+dth1.*dth5.*l_c1.*m1.*t19.*2.0+dth3.*dth5.*l_c1.*m1.*t21.*2.0+dth1.*dth2.*l_c2.*m2.*t26.*2.0+dth1.*dth5.*l_c2.*m2.*t26.*2.0+dth2.*dth5.*l_c2.*m2.*t26.*2.0+dth3.*dth4.*l_c2.*m2.*t27.*2.0+dth3.*dth5.*l_c2.*m2.*t27.*2.0+dth4.*dth5.*l_c2.*m2.*t27.*2.0;0.0];
Corr_Joint_Sp = [mt1;mt2;mt3];
end