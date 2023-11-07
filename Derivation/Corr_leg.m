function Corr_Joint_Sp = Corr_leg(in1,in2)
%Corr_leg
%    Corr_Joint_Sp = Corr_leg(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    07-Nov-2023 12:00:39

b = in2(17,:);
dth1 = in1(9,:);
dth2 = in1(10,:);
dth3 = in1(11,:);
dth4 = in1(12,:);
dth5 = in1(13,:);
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
mb = in2(6,:);
phiU = in2(29,:);
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
t10 = th3+th4;
t11 = th3+th5;
t12 = dth1.^2;
t13 = dth2.^2;
t14 = dth3.^2;
t15 = dth4.^2;
t16 = dth5.^2;
t23 = -th5;
t17 = cos(t9);
t18 = cos(t10);
t19 = cos(t11);
t20 = sin(t11);
t21 = t10+th1;
t22 = t10+th5;
t27 = t23+th1;
t30 = t9+t23;
t33 = l_AC.*l_C_m4.*m4.*t5.*t13;
t34 = l_AC.*l_C_m4.*m4.*t6.*t15;
t35 = l_AC.*l_OA.*m4.*t5.*t13;
t36 = l_AC.*l_OA.*m4.*t6.*t15;
t37 = l_A_m3.*l_OA.*m3.*t5.*t13;
t38 = l_A_m3.*l_OA.*m3.*t6.*t15;
t39 = l_B_m2.*l_OB.*m2.*t5.*t13;
t40 = dth1.*dth2.*l_AC.*l_C_m4.*m4.*t5.*2.0;
t41 = dth2.*dth5.*l_AC.*l_C_m4.*m4.*t5.*2.0;
t42 = dth3.*dth4.*l_AC.*l_C_m4.*m4.*t6.*2.0;
t43 = dth4.*dth5.*l_AC.*l_C_m4.*m4.*t6.*2.0;
t44 = dth1.*dth2.*l_AC.*l_OA.*m4.*t5.*2.0;
t45 = dth2.*dth5.*l_AC.*l_OA.*m4.*t5.*2.0;
t46 = dth3.*dth4.*l_AC.*l_OA.*m4.*t6.*2.0;
t47 = dth4.*dth5.*l_AC.*l_OA.*m4.*t6.*2.0;
t48 = dth1.*dth2.*l_A_m3.*l_OA.*m3.*t5.*2.0;
t49 = dth2.*dth5.*l_A_m3.*l_OA.*m3.*t5.*2.0;
t50 = dth3.*dth4.*l_A_m3.*l_OA.*m3.*t6.*2.0;
t51 = dth4.*dth5.*l_A_m3.*l_OA.*m3.*t6.*2.0;
t52 = dth1.*dth2.*l_B_m2.*l_OB.*m2.*t5.*2.0;
t53 = dth2.*dth5.*l_B_m2.*l_OB.*m2.*t5.*2.0;
t24 = cos(t22);
t25 = sin(t21);
t26 = sin(t22);
t28 = cos(t27);
t29 = sin(t27);
t31 = cos(t30);
t32 = sin(t30);
t54 = -t42;
t55 = -t43;
t56 = -t46;
t57 = -t47;
t58 = -t50;
t59 = -t51;
t60 = b.*l_AC.*m4.*t16.*t17;
t61 = b.*l_AC.*m4.*t16.*t18;
t62 = b.*l_A_m3.*m3.*t16.*t17;
t63 = b.*l_A_m3.*m3.*t16.*t18;
t64 = b.*l_B_m2.*m2.*t16.*t17;
t65 = b.*l_B_m2.*m2.*t16.*t18;
t66 = -t34;
t67 = -t36;
t68 = -t38;
t69 = l_B_m2.*l_OB.*m2.*t12.*t25;
t70 = l_B_m2.*l_OB.*m2.*t14.*t25;
t71 = l_B_m2.*l_OB.*m2.*t15.*t25;
t72 = l_B_m2.*l_OB.*m2.*t16.*t25;
t73 = dth1.*dth5.*l_B_m2.*l_OB.*m2.*t25.*2.0;
t74 = dth3.*dth4.*l_B_m2.*l_OB.*m2.*t25.*2.0;
t75 = dth3.*dth5.*l_B_m2.*l_OB.*m2.*t25.*2.0;
t76 = dth4.*dth5.*l_B_m2.*l_OB.*m2.*t25.*2.0;
t77 = -t61;
t78 = -t63;
t79 = -t65;
t80 = -t73;
et1 = t33+t35+t37+t39+t40-t41+t44-t45+t48-t49+t52-t53+t54+t55+t56+t57+t58+t59+t66+t67+t68+t69-t70-t71-t74-t75-t76+t80+b.*l_AC.*m4.*t12.*t17+b.*l_AC.*m4.*t13.*t17+b.*l_AC.*m4.*t14.*t18+b.*l_AC.*m4.*t15.*t18+b.*l_A_m3.*m3.*t12.*t17+b.*l_A_m3.*m3.*t13.*t17+b.*l_A_m3.*m3.*t14.*t18+b.*l_A_m3.*m3.*t15.*t18+b.*l_B_m2.*m2.*t12.*t17+b.*l_B_m2.*m2.*t13.*t17+b.*l_B_m2.*m2.*t14.*t18+b.*l_B_m2.*m2.*t15.*t18+b.*l_C_m4.*m4.*t2.*t12+b.*l_C_m4.*m4.*t3.*t14+b.*l_OA.*m3.*t2.*t12+b.*l_OB.*m2.*t2.*t12.*2.0+b.*l_OA.*m4.*t2.*t12+b.*l_OA.*m3.*t3.*t14+b.*l_OA.*m4.*t3.*t14+b.*l_O_m1.*m1.*t2.*t12+b.*l_O_m1.*m1.*t3.*t14+b.*dth1.*dth2.*l_AC.*m4.*t17.*2.0-b.*dth1.*dth5.*l_AC.*m4.*t17.*2.0;
et2 = b.*dth2.*dth5.*l_AC.*m4.*t17.*-2.0+b.*dth3.*dth4.*l_AC.*m4.*t18.*2.0+b.*dth3.*dth5.*l_AC.*m4.*t18.*2.0+b.*dth4.*dth5.*l_AC.*m4.*t18.*2.0+b.*dth1.*dth2.*l_A_m3.*m3.*t17.*2.0-b.*dth1.*dth5.*l_A_m3.*m3.*t17.*2.0-b.*dth2.*dth5.*l_A_m3.*m3.*t17.*2.0+b.*dth3.*dth4.*l_A_m3.*m3.*t18.*2.0+b.*dth3.*dth5.*l_A_m3.*m3.*t18.*2.0+b.*dth4.*dth5.*l_A_m3.*m3.*t18.*2.0+b.*dth1.*dth2.*l_B_m2.*m2.*t17.*2.0-b.*dth1.*dth5.*l_B_m2.*m2.*t17.*2.0-b.*dth2.*dth5.*l_B_m2.*m2.*t17.*2.0+b.*dth3.*dth4.*l_B_m2.*m2.*t18.*2.0+b.*dth3.*dth5.*l_B_m2.*m2.*t18.*2.0+b.*dth4.*dth5.*l_B_m2.*m2.*t18.*2.0-b.*dth1.*dth5.*l_C_m4.*m4.*t2.*2.0+b.*dth3.*dth5.*l_C_m4.*m4.*t3.*2.0-b.*dth1.*dth5.*l_OA.*m3.*t2.*2.0-b.*dth1.*dth5.*l_OB.*m2.*t2.*4.0-b.*dth1.*dth5.*l_OA.*m4.*t2.*2.0+b.*dth3.*dth5.*l_OA.*m3.*t3.*2.0+b.*dth3.*dth5.*l_OA.*m4.*t3.*2.0;
et3 = b.*dth1.*dth5.*l_O_m1.*m1.*t2.*-2.0+b.*dth3.*dth5.*l_O_m1.*m1.*t3.*2.0;
et4 = b.*m1.*t4.*t16.*-2.0-b.*m2.*t4.*t16.*2.0-b.*m3.*t4.*t16.*2.0-b.*m4.*t4.*t16.*2.0-l_AC.*m4.*t14.*t26-l_AC.*m4.*t15.*t26-l_AC.*m4.*t16.*t26+l_AC.*m4.*t12.*t32+l_AC.*m4.*t13.*t32+l_AC.*m4.*t16.*t32-l_A_m3.*m3.*t14.*t26-l_A_m3.*m3.*t15.*t26-l_A_m3.*m3.*t16.*t26+l_A_m3.*m3.*t12.*t32+l_A_m3.*m3.*t13.*t32+l_A_m3.*m3.*t16.*t32-l_B_m2.*m2.*t14.*t26-l_B_m2.*m2.*t15.*t26-l_B_m2.*m2.*t16.*t26+l_B_m2.*m2.*t12.*t32+l_B_m2.*m2.*t13.*t32+l_B_m2.*m2.*t16.*t32-l_C_m4.*m4.*t14.*t20-l_C_m4.*m4.*t16.*t20+l_C_m4.*m4.*t12.*t29+l_C_m4.*m4.*t16.*t29-l_OA.*m3.*t14.*t20-l_OA.*m4.*t14.*t20-l_OA.*m3.*t16.*t20-l_OA.*m4.*t16.*t20+l_OA.*m3.*t12.*t29+l_OB.*m2.*t12.*t29.*2.0;
et5 = l_OA.*m4.*t12.*t29+l_OA.*m3.*t16.*t29+l_OB.*m2.*t16.*t29.*2.0+l_OA.*m4.*t16.*t29-l_O_m1.*m1.*t14.*t20-l_O_m1.*m1.*t16.*t20+l_O_m1.*m1.*t12.*t29+l_O_m1.*m1.*t16.*t29-l_cb.*mb.*t4.*t16-lU.*mU.*t16.*cos(t8)-dth3.*dth4.*l_AC.*m4.*t26.*2.0-dth3.*dth5.*l_AC.*m4.*t26.*2.0+dth1.*dth2.*l_AC.*m4.*t32.*2.0-dth4.*dth5.*l_AC.*m4.*t26.*2.0-dth1.*dth5.*l_AC.*m4.*t32.*2.0-dth2.*dth5.*l_AC.*m4.*t32.*2.0-dth3.*dth4.*l_A_m3.*m3.*t26.*2.0-dth3.*dth5.*l_A_m3.*m3.*t26.*2.0+dth1.*dth2.*l_A_m3.*m3.*t32.*2.0-dth4.*dth5.*l_A_m3.*m3.*t26.*2.0-dth1.*dth5.*l_A_m3.*m3.*t32.*2.0-dth2.*dth5.*l_A_m3.*m3.*t32.*2.0-dth3.*dth4.*l_B_m2.*m2.*t26.*2.0-dth3.*dth5.*l_B_m2.*m2.*t26.*2.0+dth1.*dth2.*l_B_m2.*m2.*t32.*2.0-dth4.*dth5.*l_B_m2.*m2.*t26.*2.0-dth1.*dth5.*l_B_m2.*m2.*t32.*2.0;
et6 = dth2.*dth5.*l_B_m2.*m2.*t32.*-2.0-dth3.*dth5.*l_C_m4.*m4.*t20.*2.0-dth1.*dth5.*l_C_m4.*m4.*t29.*2.0-dth3.*dth5.*l_OA.*m3.*t20.*2.0-dth3.*dth5.*l_OA.*m4.*t20.*2.0-dth1.*dth5.*l_OA.*m3.*t29.*2.0-dth1.*dth5.*l_OB.*m2.*t29.*4.0-dth1.*dth5.*l_OA.*m4.*t29.*2.0-dth3.*dth5.*l_O_m1.*m1.*t20.*2.0-dth1.*dth5.*l_O_m1.*m1.*t29.*2.0;
et7 = b.*m1.*t7.*t16.*-2.0-b.*m2.*t7.*t16.*2.0-b.*m3.*t7.*t16.*2.0-b.*m4.*t7.*t16.*2.0+l_AC.*m4.*t14.*t24+l_AC.*m4.*t15.*t24+l_AC.*m4.*t16.*t24+l_AC.*m4.*t12.*t31+l_AC.*m4.*t13.*t31+l_AC.*m4.*t16.*t31+l_A_m3.*m3.*t14.*t24+l_A_m3.*m3.*t15.*t24+l_A_m3.*m3.*t16.*t24+l_A_m3.*m3.*t12.*t31+l_A_m3.*m3.*t13.*t31+l_A_m3.*m3.*t16.*t31+l_B_m2.*m2.*t14.*t24+l_B_m2.*m2.*t15.*t24+l_B_m2.*m2.*t16.*t24+l_B_m2.*m2.*t12.*t31+l_B_m2.*m2.*t13.*t31+l_B_m2.*m2.*t16.*t31+l_C_m4.*m4.*t14.*t19+l_C_m4.*m4.*t16.*t19+l_C_m4.*m4.*t12.*t28+l_C_m4.*m4.*t16.*t28+l_OA.*m3.*t14.*t19+l_OA.*m4.*t14.*t19+l_OA.*m3.*t16.*t19+l_OA.*m4.*t16.*t19+l_OA.*m3.*t12.*t28+l_OB.*m2.*t12.*t28.*2.0+l_OA.*m4.*t12.*t28+l_OA.*m3.*t16.*t28+l_OB.*m2.*t16.*t28.*2.0+l_OA.*m4.*t16.*t28+l_O_m1.*m1.*t14.*t19+l_O_m1.*m1.*t16.*t19;
et8 = l_O_m1.*m1.*t12.*t28+l_O_m1.*m1.*t16.*t28-l_cb.*mb.*t7.*t16-lU.*mU.*t16.*sin(t8)+dth3.*dth4.*l_AC.*m4.*t24.*2.0+dth3.*dth5.*l_AC.*m4.*t24.*2.0+dth4.*dth5.*l_AC.*m4.*t24.*2.0+dth1.*dth2.*l_AC.*m4.*t31.*2.0-dth1.*dth5.*l_AC.*m4.*t31.*2.0-dth2.*dth5.*l_AC.*m4.*t31.*2.0+dth3.*dth4.*l_A_m3.*m3.*t24.*2.0+dth3.*dth5.*l_A_m3.*m3.*t24.*2.0+dth4.*dth5.*l_A_m3.*m3.*t24.*2.0+dth1.*dth2.*l_A_m3.*m3.*t31.*2.0-dth1.*dth5.*l_A_m3.*m3.*t31.*2.0-dth2.*dth5.*l_A_m3.*m3.*t31.*2.0+dth3.*dth4.*l_B_m2.*m2.*t24.*2.0+dth3.*dth5.*l_B_m2.*m2.*t24.*2.0+dth4.*dth5.*l_B_m2.*m2.*t24.*2.0+dth1.*dth2.*l_B_m2.*m2.*t31.*2.0-dth1.*dth5.*l_B_m2.*m2.*t31.*2.0-dth2.*dth5.*l_B_m2.*m2.*t31.*2.0+dth3.*dth5.*l_C_m4.*m4.*t19.*2.0-dth1.*dth5.*l_C_m4.*m4.*t28.*2.0+dth3.*dth5.*l_OA.*m3.*t19.*2.0+dth3.*dth5.*l_OA.*m4.*t19.*2.0-dth1.*dth5.*l_OA.*m3.*t28.*2.0;
et9 = dth1.*dth5.*l_OB.*m2.*t28.*-4.0-dth1.*dth5.*l_OA.*m4.*t28.*2.0+dth3.*dth5.*l_O_m1.*m1.*t19.*2.0-dth1.*dth5.*l_O_m1.*m1.*t28.*2.0;
mt1 = [-t33-t35-t37-t39-t40+t41-t44+t45-t48+t49-t52+t53+t60+t62+t64+t70+t71+t72+t74+t75+t76+b.*l_C_m4.*m4.*t2.*t16+b.*l_OA.*m3.*t2.*t16+b.*l_OB.*m2.*t2.*t16.*2.0+b.*l_OA.*m4.*t2.*t16+b.*l_O_m1.*m1.*t2.*t16;t60+t62+t64+l_AC.*l_C_m4.*m4.*t5.*t12+l_AC.*l_C_m4.*m4.*t5.*t16+l_AC.*l_OA.*m4.*t5.*t12+l_AC.*l_OA.*m4.*t5.*t16+l_A_m3.*l_OA.*m3.*t5.*t12+l_A_m3.*l_OA.*m3.*t5.*t16+l_B_m2.*l_OB.*m2.*t5.*t12+l_B_m2.*l_OB.*m2.*t5.*t16-dth1.*dth5.*l_AC.*l_C_m4.*m4.*t5.*2.0-dth1.*dth5.*l_AC.*l_OA.*m4.*t5.*2.0-dth1.*dth5.*l_A_m3.*l_OA.*m3.*t5.*2.0-dth1.*dth5.*l_B_m2.*l_OB.*m2.*t5.*2.0];
mt2 = [t54+t55+t56+t57+t58+t59+t66+t67+t68+t69+t72+t77+t78+t79+t80-b.*l_C_m4.*m4.*t3.*t16-b.*l_OA.*m3.*t3.*t16-b.*l_OA.*m4.*t3.*t16-b.*l_O_m1.*m1.*t3.*t16;t69+t72+t77+t78+t79+t80+l_AC.*l_C_m4.*m4.*t6.*t14+l_AC.*l_C_m4.*m4.*t6.*t16+l_AC.*l_OA.*m4.*t6.*t14+l_AC.*l_OA.*m4.*t6.*t16+l_A_m3.*l_OA.*m3.*t6.*t14+l_A_m3.*l_OA.*m3.*t6.*t16+dth3.*dth5.*l_AC.*l_C_m4.*m4.*t6.*2.0+dth3.*dth5.*l_AC.*l_OA.*m4.*t6.*2.0+dth3.*dth5.*l_A_m3.*l_OA.*m3.*t6.*2.0;et1+et2+et3;et4+et5+et6;et7+et8+et9;0.0];
Corr_Joint_Sp = [mt1;mt2];
