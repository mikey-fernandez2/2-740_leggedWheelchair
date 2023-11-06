function E = energy_leggedWheelchair(in1,in2)
%energy_leggedWheelchair
%    E = energy_leggedWheelchair(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    06-Nov-2023 17:21:15

I1 = in2(7,:);
I2 = in2(8,:);
I3 = in2(9,:);
I4 = in2(10,:);
I_A = in2(11,:);
I_B = in2(12,:);
I_U = in2(27,:);
Ir = in2(24,:);
N = in2(25,:);
b = in2(17,:);
dphi = in1(16,:);
dth1 = in1(9,:);
dth2 = in1(10,:);
dth3 = in1(11,:);
dth4 = in1(12,:);
dth5 = in1(13,:);
dx = in1(14,:);
dy = in1(15,:);
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
y = in1(7,:);
t2 = cos(th5);
t3 = sin(th5);
t4 = dth3+dth5;
t5 = phiU+th5;
t6 = th1+th5;
t7 = th3+th5;
t8 = dth5.^2;
t19 = -dth5;
t20 = -dx;
t9 = dth4+t4;
t10 = b.*t2;
t11 = b.*t3;
t12 = cos(t6);
t13 = cos(t7);
t14 = sin(t5);
t15 = sin(t6);
t16 = sin(t7);
t17 = t6+th2;
t18 = t7+th4;
t25 = t4.^2;
t36 = dth1+t19;
t21 = cos(t17);
t22 = cos(t18);
t23 = sin(t17);
t24 = sin(t18);
t26 = l_C_m4.*t12;
t27 = l_C_m4.*t13;
t28 = l_OA.*t12;
t29 = l_OB.*t12;
t30 = l_OA.*t13;
t31 = l_C_m4.*t15;
t32 = l_C_m4.*t16;
t33 = l_OA.*t15;
t34 = l_OB.*t15;
t35 = l_OA.*t16;
t37 = -t11;
t38 = t9.^2;
t44 = dth2+t36;
t52 = t36.^2;
t39 = l_AC.*t21;
t40 = l_AC.*t22;
t41 = l_A_m3.*t21;
t42 = l_A_m3.*t22;
t43 = l_B_m2.*t21;
t45 = l_AC.*t23;
t46 = l_AC.*t24;
t47 = l_A_m3.*t23;
t48 = l_A_m3.*t24;
t49 = l_B_m2.*t23;
t53 = t44.^2;
t50 = dth4.*t40;
t51 = dth4.*t46;
t54 = t27+t30+t40;
t55 = t32+t35+t46;
t56 = dth3.*t55;
t57 = dth3.*t54;
t58 = t10+t55;
t60 = t37+t54;
t59 = dth5.*t58;
t61 = dth5.*t60;
t62 = dy+t51+t56+t59;
t63 = dx+t50+t57+t61;
et1 = (m3.*((dy+dth1.*(t33+t47)+dth2.*t47+dth5.*(t10+t33+t47)).^2+(t20+dth1.*(t28+t41)+dth2.*t41+dth5.*(t11+t28+t41)).^2))./2.0+(m2.*((dy+dth1.*(t34+t49)+dth2.*t49+dth5.*(t10+t34+t49)).^2+(t20+dth1.*(t29+t43)+dth2.*t43+dth5.*(t11+t29+t43)).^2))./2.0+(m2.*((dy+dth1.*t34+dth5.*(t10+t34+l_B_m2.*t24)+dth3.*l_B_m2.*t24+dth4.*l_B_m2.*t24).^2+(dx-dth1.*t29+t19.*(t11+t29-l_B_m2.*t22)+dth3.*l_B_m2.*t22+dth4.*l_B_m2.*t22).^2))./2.0+(I1.*t25)./2.0+(I4.*t25)./2.0+(I2.*t38)./2.0+(I3.*t38)./2.0+(I1.*t52)./2.0+(I2.*t53)./2.0+(I3.*t53)./2.0+(I4.*t52)./2.0+(I_B.*t8)./2.0+(I_U.*t8)./2.0;
et2 = (mU.*((dy+dth5.*lU.*cos(t5)).^2+(dx+lU.*t14.*t19).^2))./2.0+(I_A.*dphi.^2)./2.0+g.*(m1.*t11.*-2.0-m2.*t11.*2.0-m3.*t11.*2.0-m4.*t11.*2.0+m4.*t26+m2.*t29.*2.0+m3.*t28+m4.*t27+m4.*t28+m3.*t30+m4.*t30+m4.*t39+m3.*t41+m4.*t40+m2.*t43+m3.*t42-m1.*y.*2.0-m2.*y.*2.0-m3.*y.*2.0-m4.*y.*2.0+mU.*y+ma.*y+mb.*y+l_B_m2.*m2.*t22+l_O_m1.*m1.*t12+l_O_m1.*m1.*t13+lU.*mU.*t14+l_cb.*mb.*t3)+(m1.*((dy+dth5.*(t10+l_O_m1.*t15)+dth1.*l_O_m1.*t15).^2+(t20+dth5.*(t11+l_O_m1.*t12)+dth1.*l_O_m1.*t12).^2))./2.0;
et3 = (m1.*((dy+dth5.*(t10+l_O_m1.*t16)+dth3.*l_O_m1.*t16).^2+(dx+t19.*(t11-l_O_m1.*t13)+dth3.*l_O_m1.*t13).^2))./2.0+(ma.*(dx.^2+dy.^2))./2.0+(mb.*((dy+dth5.*l_cb.*t2).^2+(dx+l_cb.*t3.*t19).^2))./2.0+(m4.*(t62.^2+t63.^2))./2.0+(m4.*((dy+dth2.*t45+dth1.*(t31+t33+t45)+dth5.*(t10+t31+t33+t45)).^2+(t20+dth2.*t39+dth1.*(t26+t28+t39)+dth5.*(t11+t26+t28+t39)).^2))./2.0+(Ir.*(dth5-N.*dth1).^2)./2.0+(Ir.*(dth5+N.*dth3).^2)./2.0+(m3.*(t63.*(dx+dth3.*(t30+t42)+dth4.*t42+dth5.*(t30+t37+t42))+t62.*(dy+dth3.*(t35+t48)+dth4.*t48+dth5.*(t10+t35+t48))))./2.0;
et4 = (Ir.*(t4+N.*dth4).^2)./2.0+(Ir.*(t36+N.*dth2).^2)./2.0;
E = et1+et2+et3+et4;
