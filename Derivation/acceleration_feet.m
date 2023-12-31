function ddrFeet = acceleration_feet(in1,in2,in3)
%ACCELERATION_FEET
%    ddrFeet = ACCELERATION_FEET(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    01-Dec-2023 15:01:41

b = in3(17,:);
ddth1 = in2(9,:);
ddth2 = in2(10,:);
ddth3 = in2(11,:);
ddth4 = in2(12,:);
ddth5 = in2(13,:);
ddx = in2(14,:);
ddy = in2(15,:);
dth1 = in1(9,:);
dth2 = in1(10,:);
dth3 = in1(11,:);
dth4 = in1(12,:);
dth5 = in1(13,:);
l_AC = in3(15,:);
l_DE = in3(16,:);
l_OB = in3(14,:);
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
t18 = t10+th1;
t11 = cos(t9);
t12 = sin(t9);
t13 = l_DE.*t7;
t14 = l_OB.*t7;
t15 = l_DE.*t8;
t16 = l_OB.*t8;
t17 = -t5;
t19 = -t6;
t20 = cos(t18);
t21 = sin(t18);
t22 = t18+th2;
t23 = l_AC.*t11;
t24 = l_AC.*t12;
t27 = cos(t22);
t28 = sin(t22);
t29 = l_DE.*t20;
t30 = l_OB.*t20;
t31 = l_DE.*t21;
t32 = l_OB.*t21;
t25 = dth4.*t23;
t26 = dth4.*t24;
t33 = l_AC.*t27;
t34 = l_AC.*t28;
t37 = t13+t14+t23;
t38 = t15+t16+t24;
t35 = dth2.*t33;
t36 = dth2.*t34;
t39 = dth3.*t38;
t40 = dth3.*t37;
t41 = t5+t38;
t42 = t19+t37;
t43 = t29+t30+t33;
t44 = t31+t32+t34;
t45 = dth1.*t43;
t46 = dth1.*t44;
t47 = t19+t43;
t48 = t17+t44;
ddrFeet = reshape([ddx+dth2.*(t36+dth1.*t34-dth5.*t34)-ddth2.*t33-ddth1.*t43+ddth5.*t47+dth1.*(t36+t46-dth5.*t44)-dth5.*(t36+t46-dth5.*t48),ddy+dth2.*(t35+dth1.*t33-dth5.*t33)+ddth2.*t34+ddth1.*t44-ddth5.*t48+dth1.*(t35+t45-dth5.*t43)-dth5.*(t35+t45-dth5.*t47),ddx-dth4.*(t26+dth3.*t24+dth5.*t24)+ddth4.*t23+ddth3.*t37+ddth5.*t42-dth3.*(t26+t39+dth5.*t38)-dth5.*(t26+t39+dth5.*t41),ddy+dth4.*(t25+dth3.*t23+dth5.*t23)+ddth4.*t24+ddth3.*t38+ddth5.*t41+dth3.*(t25+t40+dth5.*t37)+dth5.*(t25+t40+dth5.*t42)],[2,2]);
