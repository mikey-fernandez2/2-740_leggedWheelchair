function b = b_leggedWheelchair(in1,in2,in3)
%b_leggedWheelchair
%    B = b_leggedWheelchair(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    02-Nov-2023 00:24:41

b = in3(17,:);
dth1 = in1(9,:);
dth2 = in1(10,:);
dth3 = in1(11,:);
dth4 = in1(12,:);
dth5 = in1(13,:);
g = in3(30,:);
lU = in3(28,:);
l_AC = in3(15,:);
l_A_m3 = in3(20,:);
l_B_m2 = in3(19,:);
l_C_m4 = in3(21,:);
l_OA = in3(13,:);
l_OB = in3(14,:);
l_O_m1 = in3(18,:);
l_cb = in3(22,:);
m1 = in3(1,:);
m2 = in3(2,:);
m3 = in3(3,:);
m4 = in3(4,:);
mU = in3(26,:);
ma = in3(5,:);
mb = in3(6,:);
phiU = in3(29,:);
tau1 = in2(1,:);
tau2 = in2(2,:);
tau3 = in2(3,:);
tau4 = in2(4,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
t2 = cos(th3);
t3 = cos(th5);
t4 = sin(th2);
t5 = sin(th4);
t6 = sin(th5);
t7 = phiU+th5;
t8 = th1+th5;
t9 = th3+th4;
t10 = th3+th5;
t11 = dth1.^2;
t12 = dth2.^2;
t13 = dth3.^2;
t14 = dth4.^2;
t15 = dth5.^2;
t17 = cos(t7);
t18 = cos(t8);
t19 = cos(t9);
t20 = cos(t10);
t21 = sin(t8);
t22 = sin(t10);
t23 = t8+th2;
t24 = t9+th5;
t29 = t8+th5;
t35 = dth3.*dth4.*l_A_m3.*l_C_m4.*m3.*t5;
t36 = dth4.*dth5.*l_A_m3.*l_C_m4.*m3.*t5;
t37 = dth3.*dth4.*l_AC.*l_OA.*m3.*t5;
t38 = dth4.*dth5.*l_AC.*l_OA.*m3.*t5;
t39 = dth3.*dth4.*l_A_m3.*l_OA.*m3.*t5;
t40 = dth4.*dth5.*l_A_m3.*l_OA.*m3.*t5;
t49 = l_AC.*l_C_m4.*m4.*t4.*t12;
t50 = l_AC.*l_C_m4.*m4.*t5.*t14;
t51 = l_AC.*l_OA.*m4.*t4.*t12;
t52 = l_AC.*l_OA.*m4.*t5.*t14;
t53 = l_A_m3.*l_OA.*m3.*t4.*t12;
t54 = l_B_m2.*l_OB.*m2.*t4.*t12;
t55 = dth1.*dth2.*l_AC.*l_C_m4.*m4.*t4.*2.0;
t56 = dth2.*dth5.*l_AC.*l_C_m4.*m4.*t4.*2.0;
t57 = dth3.*dth4.*l_AC.*l_C_m4.*m4.*t5.*2.0;
t58 = dth4.*dth5.*l_AC.*l_C_m4.*m4.*t5.*2.0;
t59 = dth1.*dth2.*l_AC.*l_OA.*m4.*t4.*2.0;
t60 = dth2.*dth5.*l_AC.*l_OA.*m4.*t4.*2.0;
t61 = dth3.*dth4.*l_AC.*l_OA.*m4.*t5.*2.0;
t62 = dth4.*dth5.*l_AC.*l_OA.*m4.*t5.*2.0;
t63 = dth1.*dth2.*l_A_m3.*l_OA.*m3.*t4.*2.0;
t64 = dth2.*dth5.*l_A_m3.*l_OA.*m3.*t4.*2.0;
t65 = dth1.*dth2.*l_B_m2.*l_OB.*m2.*t4.*2.0;
t66 = dth2.*dth5.*l_B_m2.*l_OB.*m2.*t4.*2.0;
t76 = (l_A_m3.*l_C_m4.*m3.*t5.*t14)./2.0;
t77 = (l_AC.*l_OA.*m3.*t5.*t14)./2.0;
t78 = (l_A_m3.*l_OA.*m3.*t5.*t14)./2.0;
t25 = cos(t23);
t26 = cos(t24);
t27 = sin(t23);
t28 = sin(t24);
t30 = cos(t29);
t31 = t23+th5;
t33 = t8+t24;
t41 = g.*l_C_m4.*m4.*t21;
t42 = g.*l_C_m4.*m4.*t22;
t43 = g.*l_OA.*m3.*t21;
t44 = g.*l_OA.*m4.*t21;
t45 = g.*l_OA.*m3.*t22;
t46 = g.*l_OA.*m4.*t22;
t47 = g.*l_O_m1.*m1.*t21;
t48 = g.*l_O_m1.*m1.*t22;
t67 = g.*l_OB.*m2.*t21.*2.0;
t74 = b.*l_AC.*m4.*t15.*t19;
t75 = b.*l_B_m2.*m2.*t15.*t19;
t79 = (b.*l_AC.*m3.*t15.*t19)./2.0;
t80 = (b.*l_A_m3.*m3.*t15.*t19)./2.0;
t32 = cos(t31);
t34 = sin(t33);
t68 = g.*l_AC.*m4.*t27;
t69 = g.*l_AC.*m4.*t28;
t70 = g.*l_A_m3.*m3.*t27;
t71 = g.*l_A_m3.*m3.*t28;
t72 = g.*l_B_m2.*m2.*t27;
t73 = g.*l_B_m2.*m2.*t28;
t81 = b.*l_AC.*m4.*t15.*t32;
t82 = b.*l_A_m3.*m3.*t15.*t32;
t83 = b.*l_B_m2.*m2.*t15.*t32;
t84 = l_B_m2.*l_OB.*m2.*t11.*t34;
t85 = l_B_m2.*l_OB.*m2.*t13.*t34;
t86 = l_B_m2.*l_OB.*m2.*t14.*t34;
t87 = l_B_m2.*l_OB.*m2.*t15.*t34;
t88 = dth1.*dth5.*l_B_m2.*l_OB.*m2.*t34.*2.0;
t89 = dth3.*dth4.*l_B_m2.*l_OB.*m2.*t34.*2.0;
t90 = dth3.*dth5.*l_B_m2.*l_OB.*m2.*t34.*2.0;
t91 = dth4.*dth5.*l_B_m2.*l_OB.*m2.*t34.*2.0;
t92 = -t88;
t93 = -t89;
t94 = -t90;
t95 = -t91;
t96 = -t81;
t97 = -t82;
t98 = -t83;
t99 = -t84;
t100 = -t85;
t101 = -t86;
t102 = -t87;
et1 = t35+t36+t37+t38+t39+t40+t41+t42+t43+t44+t45+t46+t47+t48+t49+t50+t51+t52+t53+t54+t55+t56+t57+t58+t59+t60+t61+t62+t63+t64+t65+t66+t67+t68+t69+t70+t71+t72+t73+t76+t77+t78-t81.*2.0-t82.*2.0-t83.*2.0-t87.*2.0+t92+t93+t94+t95+t99+t100+t101+tau1+tau3+b.*g.*m1.*t3.*2.0+b.*g.*m2.*t3.*2.0+b.*g.*m3.*t3.*2.0+b.*g.*m4.*t3.*2.0-g.*lU.*mU.*t17-g.*l_cb.*mb.*t3-(b.*l_AC.*m3.*t13.*t19)./2.0-(b.*l_AC.*m3.*t14.*t19)./2.0-b.*l_AC.*m4.*t13.*t19-b.*l_AC.*m4.*t14.*t19-b.*l_AC.*m4.*t11.*t32-b.*l_AC.*m4.*t12.*t32-(b.*l_A_m3.*m3.*t13.*t19)./2.0-(b.*l_A_m3.*m3.*t14.*t19)./2.0-b.*l_A_m3.*m3.*t11.*t32-b.*l_A_m3.*m3.*t12.*t32;
et2 = -b.*l_B_m2.*m2.*t13.*t19-b.*l_B_m2.*m2.*t14.*t19-b.*l_B_m2.*m2.*t11.*t32-b.*l_B_m2.*m2.*t12.*t32-(b.*l_C_m4.*m3.*t2.*t13)./2.0-b.*l_C_m4.*m4.*t2.*t13-b.*l_C_m4.*m4.*t11.*t30-b.*l_C_m4.*m4.*t15.*t30.*2.0-b.*l_OA.*m3.*t2.*t13-b.*l_OA.*m4.*t2.*t13-b.*l_OA.*m3.*t11.*t30-b.*l_OB.*m2.*t11.*t30.*2.0-b.*l_OA.*m4.*t11.*t30-b.*l_OA.*m3.*t15.*t30.*2.0-b.*l_OB.*m2.*t15.*t30.*4.0-b.*l_OA.*m4.*t15.*t30.*2.0-b.*l_O_m1.*m1.*t2.*t13-b.*l_O_m1.*m1.*t11.*t30-b.*l_O_m1.*m1.*t15.*t30.*2.0-b.*dth3.*dth4.*l_AC.*m3.*t19-b.*dth3.*dth4.*l_AC.*m4.*t19.*2.0-b.*dth3.*dth5.*l_AC.*m3.*t19-b.*dth3.*dth5.*l_AC.*m4.*t19.*2.0-b.*dth4.*dth5.*l_AC.*m3.*t19;
et3 = b.*dth4.*dth5.*l_AC.*m4.*t19.*-2.0-b.*dth1.*dth2.*l_AC.*m4.*t32.*2.0-b.*dth1.*dth5.*l_AC.*m4.*t32.*2.0-b.*dth2.*dth5.*l_AC.*m4.*t32.*2.0-b.*dth3.*dth4.*l_A_m3.*m3.*t19-b.*dth3.*dth5.*l_A_m3.*m3.*t19-b.*dth4.*dth5.*l_A_m3.*m3.*t19-b.*dth1.*dth2.*l_A_m3.*m3.*t32.*2.0-b.*dth1.*dth5.*l_A_m3.*m3.*t32.*2.0-b.*dth2.*dth5.*l_A_m3.*m3.*t32.*2.0-b.*dth3.*dth4.*l_B_m2.*m2.*t19.*2.0-b.*dth3.*dth5.*l_B_m2.*m2.*t19.*2.0-b.*dth4.*dth5.*l_B_m2.*m2.*t19.*2.0-b.*dth1.*dth2.*l_B_m2.*m2.*t32.*2.0-b.*dth1.*dth5.*l_B_m2.*m2.*t32.*2.0-b.*dth2.*dth5.*l_B_m2.*m2.*t32.*2.0-b.*dth3.*dth5.*l_C_m4.*m3.*t2-b.*dth3.*dth5.*l_C_m4.*m4.*t2.*2.0-b.*dth1.*dth5.*l_C_m4.*m4.*t30.*2.0-b.*dth3.*dth5.*l_OA.*m3.*t2.*2.0-b.*dth3.*dth5.*l_OA.*m4.*t2.*2.0-b.*dth1.*dth5.*l_OA.*m3.*t30.*2.0;
et4 = b.*dth1.*dth5.*l_OB.*m2.*t30.*-4.0-b.*dth1.*dth5.*l_OA.*m4.*t30.*2.0-b.*dth3.*dth5.*l_O_m1.*m1.*t2.*2.0-b.*dth1.*dth5.*l_O_m1.*m1.*t30.*2.0;
et5 = b.*m1.*t3.*t15.*2.0+b.*m2.*t3.*t15.*2.0+b.*m3.*t3.*t15.*2.0+b.*m4.*t3.*t15.*2.0-l_AC.*m4.*t11.*t27-l_AC.*m4.*t12.*t27+(l_AC.*m3.*t13.*t28)./2.0+(l_AC.*m3.*t14.*t28)./2.0+l_AC.*m4.*t13.*t28+(l_AC.*m3.*t15.*t28)./2.0+l_AC.*m4.*t14.*t28-l_AC.*m4.*t15.*t27+l_AC.*m4.*t15.*t28-l_A_m3.*m3.*t11.*t27-l_A_m3.*m3.*t12.*t27+(l_A_m3.*m3.*t13.*t28)./2.0+(l_A_m3.*m3.*t14.*t28)./2.0-l_A_m3.*m3.*t15.*t27+(l_A_m3.*m3.*t15.*t28)./2.0-l_B_m2.*m2.*t11.*t27-l_B_m2.*m2.*t12.*t27+l_B_m2.*m2.*t13.*t28+l_B_m2.*m2.*t14.*t28-l_B_m2.*m2.*t15.*t27+l_B_m2.*m2.*t15.*t28-l_C_m4.*m4.*t11.*t21+(l_C_m4.*m3.*t13.*t22)./2.0+l_C_m4.*m4.*t13.*t22+(l_C_m4.*m3.*t15.*t22)./2.0-l_C_m4.*m4.*t15.*t21;
et6 = l_C_m4.*m4.*t15.*t22-l_OA.*m3.*t11.*t21-l_OB.*m2.*t11.*t21.*2.0-l_OA.*m4.*t11.*t21+l_OA.*m3.*t13.*t22-l_OA.*m3.*t15.*t21+l_OA.*m4.*t13.*t22-l_OB.*m2.*t15.*t21.*2.0+l_OA.*m3.*t15.*t22-l_OA.*m4.*t15.*t21+l_OA.*m4.*t15.*t22-l_O_m1.*m1.*t11.*t21+l_O_m1.*m1.*t13.*t22-l_O_m1.*m1.*t15.*t21+l_O_m1.*m1.*t15.*t22+lU.*mU.*t15.*t17+l_cb.*mb.*t3.*t15-dth1.*dth2.*l_AC.*m4.*t27.*2.0-dth1.*dth5.*l_AC.*m4.*t27.*2.0-dth2.*dth5.*l_AC.*m4.*t27.*2.0+dth3.*dth4.*l_AC.*m3.*t28+dth3.*dth4.*l_AC.*m4.*t28.*2.0+dth3.*dth5.*l_AC.*m3.*t28+dth3.*dth5.*l_AC.*m4.*t28.*2.0+dth4.*dth5.*l_AC.*m3.*t28+dth4.*dth5.*l_AC.*m4.*t28.*2.0-dth1.*dth2.*l_A_m3.*m3.*t27.*2.0-dth1.*dth5.*l_A_m3.*m3.*t27.*2.0-dth2.*dth5.*l_A_m3.*m3.*t27.*2.0+dth3.*dth4.*l_A_m3.*m3.*t28+dth3.*dth5.*l_A_m3.*m3.*t28;
et7 = dth4.*dth5.*l_A_m3.*m3.*t28-dth1.*dth2.*l_B_m2.*m2.*t27.*2.0-dth1.*dth5.*l_B_m2.*m2.*t27.*2.0-dth2.*dth5.*l_B_m2.*m2.*t27.*2.0+dth3.*dth4.*l_B_m2.*m2.*t28.*2.0+dth3.*dth5.*l_B_m2.*m2.*t28.*2.0+dth4.*dth5.*l_B_m2.*m2.*t28.*2.0-dth1.*dth5.*l_C_m4.*m4.*t21.*2.0+dth3.*dth5.*l_C_m4.*m3.*t22+dth3.*dth5.*l_C_m4.*m4.*t22.*2.0-dth1.*dth5.*l_OA.*m3.*t21.*2.0-dth1.*dth5.*l_OB.*m2.*t21.*4.0-dth1.*dth5.*l_OA.*m4.*t21.*2.0+dth3.*dth5.*l_OA.*m3.*t22.*2.0+dth3.*dth5.*l_OA.*m4.*t22.*2.0-dth1.*dth5.*l_O_m1.*m1.*t21.*2.0+dth3.*dth5.*l_O_m1.*m1.*t22.*2.0;
et8 = g.*m1.*2.0+g.*m2.*2.0+g.*m3.*2.0+g.*m4.*2.0-g.*mU-g.*ma-g.*mb+b.*m1.*t6.*t15.*2.0+b.*m2.*t6.*t15.*2.0+b.*m3.*t6.*t15.*2.0+b.*m4.*t6.*t15.*2.0-l_AC.*m4.*t11.*t25-l_AC.*m4.*t12.*t25-(l_AC.*m3.*t13.*t26)./2.0-(l_AC.*m3.*t14.*t26)./2.0-l_AC.*m4.*t13.*t26-(l_AC.*m3.*t15.*t26)./2.0-l_AC.*m4.*t14.*t26-l_AC.*m4.*t15.*t25-l_AC.*m4.*t15.*t26-l_A_m3.*m3.*t11.*t25-l_A_m3.*m3.*t12.*t25-(l_A_m3.*m3.*t13.*t26)./2.0-(l_A_m3.*m3.*t14.*t26)./2.0-l_A_m3.*m3.*t15.*t25-(l_A_m3.*m3.*t15.*t26)./2.0-l_B_m2.*m2.*t11.*t25-l_B_m2.*m2.*t12.*t25-l_B_m2.*m2.*t13.*t26-l_B_m2.*m2.*t14.*t26;
et9 = -l_B_m2.*m2.*t15.*t25-l_B_m2.*m2.*t15.*t26-l_C_m4.*m4.*t11.*t18-(l_C_m4.*m3.*t13.*t20)./2.0-l_C_m4.*m4.*t13.*t20-l_C_m4.*m4.*t15.*t18-(l_C_m4.*m3.*t15.*t20)./2.0-l_C_m4.*m4.*t15.*t20-l_OA.*m3.*t11.*t18-l_OB.*m2.*t11.*t18.*2.0-l_OA.*m4.*t11.*t18-l_OA.*m3.*t13.*t20-l_OA.*m3.*t15.*t18-l_OB.*m2.*t15.*t18.*2.0-l_OA.*m4.*t13.*t20-l_OA.*m4.*t15.*t18-l_OA.*m3.*t15.*t20-l_OA.*m4.*t15.*t20-l_O_m1.*m1.*t11.*t18-l_O_m1.*m1.*t13.*t20-l_O_m1.*m1.*t15.*t18-l_O_m1.*m1.*t15.*t20+l_cb.*mb.*t6.*t15+lU.*mU.*t15.*sin(t7)-dth1.*dth2.*l_AC.*m4.*t25.*2.0-dth1.*dth5.*l_AC.*m4.*t25.*2.0-dth2.*dth5.*l_AC.*m4.*t25.*2.0;
et10 = -dth3.*dth4.*l_AC.*m3.*t26-dth3.*dth4.*l_AC.*m4.*t26.*2.0-dth3.*dth5.*l_AC.*m3.*t26-dth3.*dth5.*l_AC.*m4.*t26.*2.0-dth4.*dth5.*l_AC.*m3.*t26-dth4.*dth5.*l_AC.*m4.*t26.*2.0-dth1.*dth2.*l_A_m3.*m3.*t25.*2.0-dth1.*dth5.*l_A_m3.*m3.*t25.*2.0-dth2.*dth5.*l_A_m3.*m3.*t25.*2.0-dth3.*dth4.*l_A_m3.*m3.*t26-dth3.*dth5.*l_A_m3.*m3.*t26-dth4.*dth5.*l_A_m3.*m3.*t26-dth1.*dth2.*l_B_m2.*m2.*t25.*2.0-dth1.*dth5.*l_B_m2.*m2.*t25.*2.0-dth2.*dth5.*l_B_m2.*m2.*t25.*2.0-dth3.*dth4.*l_B_m2.*m2.*t26.*2.0-dth3.*dth5.*l_B_m2.*m2.*t26.*2.0-dth4.*dth5.*l_B_m2.*m2.*t26.*2.0-dth1.*dth5.*l_C_m4.*m4.*t18.*2.0-dth3.*dth5.*l_C_m4.*m3.*t20-dth3.*dth5.*l_C_m4.*m4.*t20.*2.0-dth1.*dth5.*l_OA.*m3.*t18.*2.0-dth1.*dth5.*l_OB.*m2.*t18.*4.0-dth1.*dth5.*l_OA.*m4.*t18.*2.0-dth3.*dth5.*l_OA.*m3.*t20.*2.0;
et11 = dth3.*dth5.*l_OA.*m4.*t20.*-2.0-dth1.*dth5.*l_O_m1.*m1.*t18.*2.0-dth3.*dth5.*l_O_m1.*m1.*t20.*2.0;
mt1 = [t41+t43+t44+t47+t49+t51+t53+t54+t55+t56+t59+t60+t63+t64+t65+t66+t67+t68+t70+t72+t93+t94+t95+t96+t97+t98+t100+t101+t102-tau1-b.*l_C_m4.*m4.*t15.*t30-b.*l_OA.*m3.*t15.*t30-b.*l_OB.*m2.*t15.*t30.*2.0-b.*l_OA.*m4.*t15.*t30-b.*l_O_m1.*m1.*t15.*t30;t68+t70+t72+t96+t97+t98-tau2-l_AC.*l_C_m4.*m4.*t4.*t11-l_AC.*l_C_m4.*m4.*t4.*t15-l_AC.*l_OA.*m4.*t4.*t11-l_AC.*l_OA.*m4.*t4.*t15-l_A_m3.*l_OA.*m3.*t4.*t11-l_A_m3.*l_OA.*m3.*t4.*t15-l_B_m2.*l_OB.*m2.*t4.*t11-l_B_m2.*l_OB.*m2.*t4.*t15-dth1.*dth5.*l_AC.*l_C_m4.*m4.*t4.*2.0-dth1.*dth5.*l_AC.*l_OA.*m4.*t4.*2.0-dth1.*dth5.*l_A_m3.*l_OA.*m3.*t4.*2.0-dth1.*dth5.*l_B_m2.*l_OB.*m2.*t4.*2.0];
mt2 = [t35+t36+t37+t38+t39+t40+t42+t45+t46+t48+t50+t52+t57+t58+t61+t62+t69+t71+t73+t74+t75+t76+t77+t78+t79+t80+t92+t99+t102+tau3+(b.*l_C_m4.*m3.*t2.*t15)./2.0+b.*l_C_m4.*m4.*t2.*t15+b.*l_OA.*m3.*t2.*t15+b.*l_OA.*m4.*t2.*t15+b.*l_O_m1.*m1.*t2.*t15];
mt3 = [t69+t71+t73+t74+t75+t79+t80+t92+t99+t102+tau4-l_AC.*l_C_m4.*m4.*t5.*t13-l_AC.*l_C_m4.*m4.*t5.*t15-(l_A_m3.*l_C_m4.*m3.*t5.*t13)./2.0-(l_A_m3.*l_C_m4.*m3.*t5.*t15)./2.0-(l_AC.*l_OA.*m3.*t5.*t13)./2.0-l_AC.*l_OA.*m4.*t5.*t13-(l_AC.*l_OA.*m3.*t5.*t15)./2.0-l_AC.*l_OA.*m4.*t5.*t15-(l_A_m3.*l_OA.*m3.*t5.*t13)./2.0-(l_A_m3.*l_OA.*m3.*t5.*t15)./2.0-dth3.*dth5.*l_AC.*l_C_m4.*m4.*t5.*2.0-dth3.*dth5.*l_A_m3.*l_C_m4.*m3.*t5-dth3.*dth5.*l_AC.*l_OA.*m3.*t5-dth3.*dth5.*l_AC.*l_OA.*m4.*t5.*2.0-dth3.*dth5.*l_A_m3.*l_OA.*m3.*t5;et1+et2+et3+et4;et5+et6+et7;et8+et9+et10+et11;0.0];
b = [mt1;mt2;mt3];
end
