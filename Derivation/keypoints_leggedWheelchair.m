function keypoints = keypoints_leggedWheelchair(in1,in2)
%keypoints_leggedWheelchair
%    KEYPOINTS = keypoints_leggedWheelchair(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    14-Nov-2023 17:39:26

b = in2(17,:);
lU = in2(28,:);
l_AC = in2(15,:);
l_DE = in2(16,:);
l_OA = in2(13,:);
l_OB = in2(14,:);
phiU = in2(29,:);
r = in2(23,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
x = in1(6,:);
y = in1(7,:);
t2 = cos(th5);
t3 = sin(th5);
t4 = phiU+th5;
t5 = th3+th5;
t11 = -th5;
t6 = b.*t2;
t7 = b.*t3;
t8 = cos(t5);
t9 = sin(t5);
t10 = t5+th4;
t18 = t11+th1;
t12 = cos(t10);
t13 = sin(t10);
t14 = l_OA.*t8;
t15 = l_OB.*t8;
t16 = l_OA.*t9;
t17 = l_OB.*t9;
t19 = cos(t18);
t20 = sin(t18);
t21 = t18+th2;
t22 = l_AC.*t12;
t23 = l_AC.*t13;
t24 = -t14;
t25 = -t15;
t26 = cos(t21);
t27 = sin(t21);
t28 = l_OA.*t19;
t29 = l_OB.*t19;
t30 = l_OA.*t20;
t31 = l_OB.*t20;
t32 = -t22;
t33 = l_AC.*t26;
t34 = l_AC.*t27;
t35 = -t28;
t36 = -t29;
t37 = -t30;
t38 = -t31;
t39 = -t33;
t40 = -t34;
keypoints = reshape([x,y,t6+x,t7+y,t6+t37+x,t7+t35+y,t6+t38+x,t7+t36+y,t6+t37+t40+x,t7+t35+t39+y,t6+t38+t40+x,t7+t36+t39+y,t6+t38+t40+x-l_DE.*t20,t7+t36+t39+y-l_DE.*t19,t6+t16+x,t7+t24+y,t6+t17+x,t7+t25+y,t6+t16+t23+x,t7+t24+t32+y,t6+t17+t23+x,t7+t25+t32+y,t6+t17+t23+x+l_DE.*t9,t7+t25+t32+y-l_DE.*t8,x+lU.*cos(t4),y+lU.*sin(t4),x,-r+y],[2,14]);
