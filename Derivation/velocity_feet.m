function drFeet = velocity_feet(in1,in2)
%VELOCITY_FEET
%    drFeet = VELOCITY_FEET(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    30-Oct-2023 20:51:57

b = in2(16,:);
dth1 = in1(9,:);
dth2 = in1(10,:);
dth3 = in1(11,:);
dth4 = in1(12,:);
dth5 = in1(13,:);
dx = in1(14,:);
dy = in1(15,:);
l1 = in2(14,:);
l2 = in2(15,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
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
t14 = cos(t12);
t15 = cos(t13);
t16 = sin(t12);
t17 = sin(t13);
t18 = l1.*t8;
t19 = l1.*t9;
t20 = l1.*t10;
t21 = l1.*t11;
t22 = -t7;
t23 = l2.*t14;
t24 = l2.*t15;
t25 = l2.*t16;
t26 = l2.*t17;
drFeet = reshape([dx+dth1.*(t18+t23)+dth2.*t23+dth5.*(t18+t22+t23),dy+dth1.*(t20+t25)+dth2.*t25+dth5.*(t6+t20+t25),dx+dth3.*(t19+t24)+dth4.*t24+dth5.*(t19+t22+t24),dy+dth3.*(t21+t26)+dth4.*t26+dth5.*(t6+t21+t26)],[2,2]);
end
