function Corr_Joint_Sp = Corr_leg(in1,in2)
%Corr_leg
%    Corr_Joint_Sp = Corr_leg(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    02-Nov-2023 00:24:47

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
t17 = cos(t8);
t18 = cos(t9);
t19 = cos(t10);
t20 = sin(t8);
t21 = sin(t10);
t22 = t8+th2;
t23 = t9+th5;
t28 = t8+th5;
t34 = dth3.*dth4.*l_A_m3.*l_C_m4.*m3.*t5;
t35 = dth4.*dth5.*l_A_m3.*l_C_m4.*m3.*t5;
t36 = dth3.*dth4.*l_AC.*l_OA.*m3.*t5;
t37 = dth4.*dth5.*l_AC.*l_OA.*m3.*t5;
t38 = dth3.*dth4.*l_A_m3.*l_OA.*m3.*t5;
t39 = dth4.*dth5.*l_A_m3.*l_OA.*m3.*t5;
t40 = l_AC.*l_C_m4.*m4.*t4.*t12;
t41 = l_AC.*l_C_m4.*m4.*t5.*t14;
t42 = l_AC.*l_OA.*m4.*t4.*t12;
t43 = l_AC.*l_OA.*m4.*t5.*t14;
t44 = l_A_m3.*l_OA.*m3.*t4.*t12;
t45 = l_B_m2.*l_OB.*m2.*t4.*t12;
t46 = dth1.*dth2.*l_AC.*l_C_m4.*m4.*t4.*2.0;
t47 = dth2.*dth5.*l_AC.*l_C_m4.*m4.*t4.*2.0;
t48 = dth3.*dth4.*l_AC.*l_C_m4.*m4.*t5.*2.0;
t49 = dth4.*dth5.*l_AC.*l_C_m4.*m4.*t5.*2.0;
t50 = dth1.*dth2.*l_AC.*l_OA.*m4.*t4.*2.0;
t51 = dth2.*dth5.*l_AC.*l_OA.*m4.*t4.*2.0;
t52 = dth3.*dth4.*l_AC.*l_OA.*m4.*t5.*2.0;
t53 = dth4.*dth5.*l_AC.*l_OA.*m4.*t5.*2.0;
t54 = dth1.*dth2.*l_A_m3.*l_OA.*m3.*t4.*2.0;
t55 = dth2.*dth5.*l_A_m3.*l_OA.*m3.*t4.*2.0;
t56 = dth1.*dth2.*l_B_m2.*l_OB.*m2.*t4.*2.0;
t57 = dth2.*dth5.*l_B_m2.*l_OB.*m2.*t4.*2.0;
t84 = (l_A_m3.*l_C_m4.*m3.*t5.*t14)./2.0;
t85 = (l_AC.*l_OA.*m3.*t5.*t14)./2.0;
t86 = (l_A_m3.*l_OA.*m3.*t5.*t14)./2.0;
t24 = cos(t22);
t25 = cos(t23);
t26 = sin(t22);
t27 = sin(t23);
t29 = cos(t28);
t30 = t22+th5;
t32 = t8+t23;
t58 = -t46;
t59 = -t47;
t60 = -t48;
t61 = -t49;
t62 = -t34;
t63 = -t35;
t64 = -t50;
t65 = -t51;
t66 = -t36;
t67 = -t52;
t68 = -t37;
t69 = -t53;
t70 = -t54;
t71 = -t55;
t72 = -t38;
t73 = -t39;
t74 = -t56;
t75 = -t57;
t76 = b.*l_AC.*m4.*t15.*t18;
t77 = b.*l_B_m2.*m2.*t15.*t18;
t78 = -t40;
t79 = -t41;
t80 = -t42;
t81 = -t43;
t82 = -t44;
t83 = -t45;
t89 = -t84;
t90 = -t85;
t91 = -t86;
t92 = (b.*l_AC.*m3.*t15.*t18)./2.0;
t93 = (b.*l_A_m3.*m3.*t15.*t18)./2.0;
t31 = cos(t30);
t33 = sin(t32);
t87 = -t76;
t88 = -t77;
t97 = -t92;
t98 = -t93;
t94 = b.*l_AC.*m4.*t15.*t31;
t95 = b.*l_A_m3.*m3.*t15.*t31;
t96 = b.*l_B_m2.*m2.*t15.*t31;
t99 = l_B_m2.*l_OB.*m2.*t11.*t33;
t100 = l_B_m2.*l_OB.*m2.*t13.*t33;
t101 = l_B_m2.*l_OB.*m2.*t14.*t33;
t102 = l_B_m2.*l_OB.*m2.*t15.*t33;
t103 = dth1.*dth5.*l_B_m2.*l_OB.*m2.*t33.*2.0;
t104 = dth3.*dth4.*l_B_m2.*l_OB.*m2.*t33.*2.0;
t105 = dth3.*dth5.*l_B_m2.*l_OB.*m2.*t33.*2.0;
t106 = dth4.*dth5.*l_B_m2.*l_OB.*m2.*t33.*2.0;
et1 = t58+t59+t60+t61+t62+t63+t64+t65+t66+t67+t68+t69+t70+t71+t72+t73+t74+t75+t78+t79+t80+t81+t82+t83+t89+t90+t91+t94.*2.0+t95.*2.0+t96.*2.0+t99+t100+t101+t102.*2.0+t103+t104+t105+t106+(b.*l_AC.*m3.*t13.*t18)./2.0+(b.*l_AC.*m3.*t14.*t18)./2.0+b.*l_AC.*m4.*t13.*t18+b.*l_AC.*m4.*t14.*t18+b.*l_AC.*m4.*t11.*t31+b.*l_AC.*m4.*t12.*t31+(b.*l_A_m3.*m3.*t13.*t18)./2.0+(b.*l_A_m3.*m3.*t14.*t18)./2.0+b.*l_A_m3.*m3.*t11.*t31+b.*l_A_m3.*m3.*t12.*t31+b.*l_B_m2.*m2.*t13.*t18+b.*l_B_m2.*m2.*t14.*t18+b.*l_B_m2.*m2.*t11.*t31+b.*l_B_m2.*m2.*t12.*t31+(b.*l_C_m4.*m3.*t2.*t13)./2.0+b.*l_C_m4.*m4.*t2.*t13+b.*l_C_m4.*m4.*t11.*t29+b.*l_C_m4.*m4.*t15.*t29.*2.0+b.*l_OA.*m3.*t2.*t13+b.*l_OA.*m4.*t2.*t13+b.*l_OA.*m3.*t11.*t29+b.*l_OB.*m2.*t11.*t29.*2.0;
et2 = b.*l_OA.*m4.*t11.*t29+b.*l_OA.*m3.*t15.*t29.*2.0+b.*l_OB.*m2.*t15.*t29.*4.0+b.*l_OA.*m4.*t15.*t29.*2.0+b.*l_O_m1.*m1.*t2.*t13+b.*l_O_m1.*m1.*t11.*t29+b.*l_O_m1.*m1.*t15.*t29.*2.0+b.*dth3.*dth4.*l_AC.*m3.*t18+b.*dth3.*dth4.*l_AC.*m4.*t18.*2.0+b.*dth3.*dth5.*l_AC.*m3.*t18+b.*dth3.*dth5.*l_AC.*m4.*t18.*2.0+b.*dth4.*dth5.*l_AC.*m3.*t18+b.*dth4.*dth5.*l_AC.*m4.*t18.*2.0+b.*dth1.*dth2.*l_AC.*m4.*t31.*2.0+b.*dth1.*dth5.*l_AC.*m4.*t31.*2.0+b.*dth2.*dth5.*l_AC.*m4.*t31.*2.0+b.*dth3.*dth4.*l_A_m3.*m3.*t18+b.*dth3.*dth5.*l_A_m3.*m3.*t18+b.*dth4.*dth5.*l_A_m3.*m3.*t18+b.*dth1.*dth2.*l_A_m3.*m3.*t31.*2.0+b.*dth1.*dth5.*l_A_m3.*m3.*t31.*2.0+b.*dth2.*dth5.*l_A_m3.*m3.*t31.*2.0+b.*dth3.*dth4.*l_B_m2.*m2.*t18.*2.0+b.*dth3.*dth5.*l_B_m2.*m2.*t18.*2.0+b.*dth4.*dth5.*l_B_m2.*m2.*t18.*2.0+b.*dth1.*dth2.*l_B_m2.*m2.*t31.*2.0+b.*dth1.*dth5.*l_B_m2.*m2.*t31.*2.0;
et3 = b.*dth2.*dth5.*l_B_m2.*m2.*t31.*2.0+b.*dth3.*dth5.*l_C_m4.*m3.*t2+b.*dth3.*dth5.*l_C_m4.*m4.*t2.*2.0+b.*dth1.*dth5.*l_C_m4.*m4.*t29.*2.0+b.*dth3.*dth5.*l_OA.*m3.*t2.*2.0+b.*dth3.*dth5.*l_OA.*m4.*t2.*2.0+b.*dth1.*dth5.*l_OA.*m3.*t29.*2.0+b.*dth1.*dth5.*l_OB.*m2.*t29.*4.0+b.*dth1.*dth5.*l_OA.*m4.*t29.*2.0+b.*dth3.*dth5.*l_O_m1.*m1.*t2.*2.0+b.*dth1.*dth5.*l_O_m1.*m1.*t29.*2.0;
et4 = b.*m1.*t3.*t15.*-2.0-b.*m2.*t3.*t15.*2.0-b.*m3.*t3.*t15.*2.0-b.*m4.*t3.*t15.*2.0+l_AC.*m4.*t11.*t26+l_AC.*m4.*t12.*t26-(l_AC.*m3.*t13.*t27)./2.0-(l_AC.*m3.*t14.*t27)./2.0-l_AC.*m4.*t13.*t27-(l_AC.*m3.*t15.*t27)./2.0-l_AC.*m4.*t14.*t27+l_AC.*m4.*t15.*t26-l_AC.*m4.*t15.*t27+l_A_m3.*m3.*t11.*t26+l_A_m3.*m3.*t12.*t26-(l_A_m3.*m3.*t13.*t27)./2.0-(l_A_m3.*m3.*t14.*t27)./2.0+l_A_m3.*m3.*t15.*t26-(l_A_m3.*m3.*t15.*t27)./2.0+l_B_m2.*m2.*t11.*t26+l_B_m2.*m2.*t12.*t26-l_B_m2.*m2.*t13.*t27-l_B_m2.*m2.*t14.*t27+l_B_m2.*m2.*t15.*t26-l_B_m2.*m2.*t15.*t27+l_C_m4.*m4.*t11.*t20-(l_C_m4.*m3.*t13.*t21)./2.0-l_C_m4.*m4.*t13.*t21-(l_C_m4.*m3.*t15.*t21)./2.0;
et5 = l_C_m4.*m4.*t15.*t20-l_C_m4.*m4.*t15.*t21+l_OA.*m3.*t11.*t20+l_OB.*m2.*t11.*t20.*2.0+l_OA.*m4.*t11.*t20-l_OA.*m3.*t13.*t21+l_OA.*m3.*t15.*t20-l_OA.*m4.*t13.*t21+l_OB.*m2.*t15.*t20.*2.0-l_OA.*m3.*t15.*t21+l_OA.*m4.*t15.*t20-l_OA.*m4.*t15.*t21+l_O_m1.*m1.*t11.*t20-l_O_m1.*m1.*t13.*t21+l_O_m1.*m1.*t15.*t20-l_O_m1.*m1.*t15.*t21-l_cb.*mb.*t3.*t15-lU.*mU.*t15.*cos(t7)+dth1.*dth2.*l_AC.*m4.*t26.*2.0+dth1.*dth5.*l_AC.*m4.*t26.*2.0+dth2.*dth5.*l_AC.*m4.*t26.*2.0-dth3.*dth4.*l_AC.*m3.*t27-dth3.*dth4.*l_AC.*m4.*t27.*2.0-dth3.*dth5.*l_AC.*m3.*t27-dth3.*dth5.*l_AC.*m4.*t27.*2.0-dth4.*dth5.*l_AC.*m3.*t27-dth4.*dth5.*l_AC.*m4.*t27.*2.0+dth1.*dth2.*l_A_m3.*m3.*t26.*2.0+dth1.*dth5.*l_A_m3.*m3.*t26.*2.0;
et6 = dth2.*dth5.*l_A_m3.*m3.*t26.*2.0-dth3.*dth4.*l_A_m3.*m3.*t27-dth3.*dth5.*l_A_m3.*m3.*t27-dth4.*dth5.*l_A_m3.*m3.*t27+dth1.*dth2.*l_B_m2.*m2.*t26.*2.0+dth1.*dth5.*l_B_m2.*m2.*t26.*2.0+dth2.*dth5.*l_B_m2.*m2.*t26.*2.0-dth3.*dth4.*l_B_m2.*m2.*t27.*2.0-dth3.*dth5.*l_B_m2.*m2.*t27.*2.0-dth4.*dth5.*l_B_m2.*m2.*t27.*2.0+dth1.*dth5.*l_C_m4.*m4.*t20.*2.0-dth3.*dth5.*l_C_m4.*m3.*t21-dth3.*dth5.*l_C_m4.*m4.*t21.*2.0+dth1.*dth5.*l_OA.*m3.*t20.*2.0+dth1.*dth5.*l_OB.*m2.*t20.*4.0+dth1.*dth5.*l_OA.*m4.*t20.*2.0-dth3.*dth5.*l_OA.*m3.*t21.*2.0-dth3.*dth5.*l_OA.*m4.*t21.*2.0+dth1.*dth5.*l_O_m1.*m1.*t20.*2.0-dth3.*dth5.*l_O_m1.*m1.*t21.*2.0;
et7 = b.*m1.*t6.*t15.*-2.0-b.*m2.*t6.*t15.*2.0-b.*m3.*t6.*t15.*2.0-b.*m4.*t6.*t15.*2.0+l_AC.*m4.*t11.*t24+l_AC.*m4.*t12.*t24+(l_AC.*m3.*t13.*t25)./2.0+(l_AC.*m3.*t14.*t25)./2.0+l_AC.*m4.*t13.*t25+(l_AC.*m3.*t15.*t25)./2.0+l_AC.*m4.*t14.*t25+l_AC.*m4.*t15.*t24+l_AC.*m4.*t15.*t25+l_A_m3.*m3.*t11.*t24+l_A_m3.*m3.*t12.*t24+(l_A_m3.*m3.*t13.*t25)./2.0+(l_A_m3.*m3.*t14.*t25)./2.0+l_A_m3.*m3.*t15.*t24+(l_A_m3.*m3.*t15.*t25)./2.0+l_B_m2.*m2.*t11.*t24+l_B_m2.*m2.*t12.*t24+l_B_m2.*m2.*t13.*t25+l_B_m2.*m2.*t14.*t25+l_B_m2.*m2.*t15.*t24+l_B_m2.*m2.*t15.*t25+l_C_m4.*m4.*t11.*t17+(l_C_m4.*m3.*t13.*t19)./2.0+l_C_m4.*m4.*t13.*t19+l_C_m4.*m4.*t15.*t17+(l_C_m4.*m3.*t15.*t19)./2.0+l_C_m4.*m4.*t15.*t19+l_OA.*m3.*t11.*t17+l_OB.*m2.*t11.*t17.*2.0;
et8 = l_OA.*m4.*t11.*t17+l_OA.*m3.*t13.*t19+l_OA.*m3.*t15.*t17+l_OB.*m2.*t15.*t17.*2.0+l_OA.*m4.*t13.*t19+l_OA.*m4.*t15.*t17+l_OA.*m3.*t15.*t19+l_OA.*m4.*t15.*t19+l_O_m1.*m1.*t11.*t17+l_O_m1.*m1.*t13.*t19+l_O_m1.*m1.*t15.*t17+l_O_m1.*m1.*t15.*t19-l_cb.*mb.*t6.*t15-lU.*mU.*t15.*sin(t7)+dth1.*dth2.*l_AC.*m4.*t24.*2.0+dth1.*dth5.*l_AC.*m4.*t24.*2.0+dth2.*dth5.*l_AC.*m4.*t24.*2.0+dth3.*dth4.*l_AC.*m3.*t25+dth3.*dth4.*l_AC.*m4.*t25.*2.0+dth3.*dth5.*l_AC.*m3.*t25+dth3.*dth5.*l_AC.*m4.*t25.*2.0+dth4.*dth5.*l_AC.*m3.*t25+dth4.*dth5.*l_AC.*m4.*t25.*2.0+dth1.*dth2.*l_A_m3.*m3.*t24.*2.0+dth1.*dth5.*l_A_m3.*m3.*t24.*2.0+dth2.*dth5.*l_A_m3.*m3.*t24.*2.0+dth3.*dth4.*l_A_m3.*m3.*t25+dth3.*dth5.*l_A_m3.*m3.*t25+dth4.*dth5.*l_A_m3.*m3.*t25+dth1.*dth2.*l_B_m2.*m2.*t24.*2.0+dth1.*dth5.*l_B_m2.*m2.*t24.*2.0+dth2.*dth5.*l_B_m2.*m2.*t24.*2.0;
et9 = dth3.*dth4.*l_B_m2.*m2.*t25.*2.0+dth3.*dth5.*l_B_m2.*m2.*t25.*2.0+dth4.*dth5.*l_B_m2.*m2.*t25.*2.0+dth1.*dth5.*l_C_m4.*m4.*t17.*2.0+dth3.*dth5.*l_C_m4.*m3.*t19+dth3.*dth5.*l_C_m4.*m4.*t19.*2.0+dth1.*dth5.*l_OA.*m3.*t17.*2.0+dth1.*dth5.*l_OB.*m2.*t17.*4.0+dth1.*dth5.*l_OA.*m4.*t17.*2.0+dth3.*dth5.*l_OA.*m3.*t19.*2.0+dth3.*dth5.*l_OA.*m4.*t19.*2.0+dth1.*dth5.*l_O_m1.*m1.*t17.*2.0+dth3.*dth5.*l_O_m1.*m1.*t19.*2.0;
mt1 = [t58+t59+t64+t65+t70+t71+t74+t75+t78+t80+t82+t83+t94+t95+t96+t100+t101+t102+t104+t105+t106+b.*l_C_m4.*m4.*t15.*t29+b.*l_OA.*m3.*t15.*t29+b.*l_OB.*m2.*t15.*t29.*2.0+b.*l_OA.*m4.*t15.*t29+b.*l_O_m1.*m1.*t15.*t29;t94+t95+t96+l_AC.*l_C_m4.*m4.*t4.*t11+l_AC.*l_C_m4.*m4.*t4.*t15+l_AC.*l_OA.*m4.*t4.*t11+l_AC.*l_OA.*m4.*t4.*t15+l_A_m3.*l_OA.*m3.*t4.*t11+l_A_m3.*l_OA.*m3.*t4.*t15+l_B_m2.*l_OB.*m2.*t4.*t11+l_B_m2.*l_OB.*m2.*t4.*t15+dth1.*dth5.*l_AC.*l_C_m4.*m4.*t4.*2.0+dth1.*dth5.*l_AC.*l_OA.*m4.*t4.*2.0+dth1.*dth5.*l_A_m3.*l_OA.*m3.*t4.*2.0+dth1.*dth5.*l_B_m2.*l_OB.*m2.*t4.*2.0];
mt2 = [t60+t61+t62+t63+t66+t67+t68+t69+t72+t73+t79+t81+t87+t88+t89+t90+t91+t97+t98+t99+t102+t103-(b.*l_C_m4.*m3.*t2.*t15)./2.0-b.*l_C_m4.*m4.*t2.*t15-b.*l_OA.*m3.*t2.*t15-b.*l_OA.*m4.*t2.*t15-b.*l_O_m1.*m1.*t2.*t15;t87+t88+t97+t98+t99+t102+t103+l_AC.*l_C_m4.*m4.*t5.*t13+l_AC.*l_C_m4.*m4.*t5.*t15+(l_A_m3.*l_C_m4.*m3.*t5.*t13)./2.0+(l_A_m3.*l_C_m4.*m3.*t5.*t15)./2.0+(l_AC.*l_OA.*m3.*t5.*t13)./2.0+l_AC.*l_OA.*m4.*t5.*t13+(l_AC.*l_OA.*m3.*t5.*t15)./2.0+l_AC.*l_OA.*m4.*t5.*t15+(l_A_m3.*l_OA.*m3.*t5.*t13)./2.0+(l_A_m3.*l_OA.*m3.*t5.*t15)./2.0+dth3.*dth5.*l_AC.*l_C_m4.*m4.*t5.*2.0+dth3.*dth5.*l_A_m3.*l_C_m4.*m3.*t5+dth3.*dth5.*l_AC.*l_OA.*m3.*t5+dth3.*dth5.*l_AC.*l_OA.*m4.*t5.*2.0+dth3.*dth5.*l_A_m3.*l_OA.*m3.*t5;et1+et2+et3;et4+et5+et6;et7+et8+et9];
mt3 = [0.0];
Corr_Joint_Sp = [mt1;mt2;mt3];
end
