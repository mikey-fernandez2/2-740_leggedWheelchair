function E = energy_leggedWheelchair(in1,in2)
%energy_leggedWheelchair
%    E = energy_leggedWheelchair(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    30-Oct-2023 20:51:56

I_1 = in2(9,:);
I_2 = in2(10,:);
I_A = in2(12,:);
I_B = in2(13,:);
I_U = in2(4,:);
Ir = in2(21,:);
N = in2(22,:);
b = in2(16,:);
dphi = in1(16,:);
dth1 = in1(9,:);
dth2 = in1(10,:);
dth3 = in1(11,:);
dth4 = in1(12,:);
dth5 = in1(13,:);
dx = in1(14,:);
dy = in1(15,:);
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
y = in1(7,:);
t2 = cos(th5);
t3 = sin(th5);
t4 = phiU+th5;
t5 = th1+th5;
t6 = th3+th5;
t7 = dth5.^2;
t8 = b.*t2;
t9 = b.*t3;
t10 = cos(t5);
t11 = cos(t6);
t12 = sin(t4);
t13 = sin(t5);
t14 = sin(t6);
t15 = t5+th2;
t16 = t6+th4;
t17 = cos(t15);
t18 = cos(t16);
t19 = sin(t15);
t20 = sin(t16);
t21 = l1.*t10;
t22 = l1.*t11;
t23 = l1.*t13;
t24 = l1.*t14;
t25 = -t9;
t26 = l_c2.*t17;
t27 = l_c2.*t18;
t28 = l_c2.*t19;
t29 = l_c2.*t20;
et1 = (I_B.*t7)./2.0+(I_U.*t7)./2.0+(mU.*((dy+dth5.*lU.*cos(t4)).^2+(dx-dth5.*lU.*t12).^2))./2.0+(I_1.*(dth1+dth5).^2)./2.0+(I_1.*(dth3+dth5).^2)./2.0+(Ir.*(dth1+dth5+N.*dth2).^2)./2.0+(Ir.*(dth3+dth5+N.*dth4).^2)./2.0+(m1.*((dx-dth5.*(t9-l_c1.*t10)+dth1.*l_c1.*t10).^2+(dy+dth5.*(t8+l_c1.*t13)+dth1.*l_c1.*t13).^2))./2.0+(m1.*((dx-dth5.*(t9-l_c1.*t11)+dth3.*l_c1.*t11).^2+(dy+dth5.*(t8+l_c1.*t14)+dth3.*l_c1.*t14).^2))./2.0+(I_A.*dphi.^2)./2.0+(I_2.*(dth1+dth2+dth5).^2)./2.0+(I_2.*(dth3+dth4+dth5).^2)./2.0;
et2 = (ma.*(dx.^2+dy.^2))./2.0-g.*(m1.*t9.*2.0+m2.*t9.*2.0-m2.*t21-m2.*t22-m2.*t26-m2.*t27+m1.*y.*2.0+m2.*y.*2.0+mU.*y+ma.*y+mb.*y-l_c1.*m1.*t10-l_c1.*m1.*t11+lU.*mU.*t12+l_cb.*mb.*t3)+(mb.*((dx-dth5.*l_cb.*t3).^2+(dy+dth5.*l_cb.*t2).^2))./2.0+(Ir.*(dth5+N.*dth1).^2)./2.0+(Ir.*(dth5+N.*dth3).^2)./2.0+(m2.*((dx+dth1.*(t21+t26)+dth2.*t26+dth5.*(t21+t25+t26)).^2+(dy+dth1.*(t23+t28)+dth2.*t28+dth5.*(t8+t23+t28)).^2))./2.0;
et3 = (m2.*((dx+dth3.*(t22+t27)+dth4.*t27+dth5.*(t22+t25+t27)).^2+(dy+dth3.*(t24+t29)+dth4.*t29+dth5.*(t8+t24+t29)).^2))./2.0;
E = et1+et2+et3;
end