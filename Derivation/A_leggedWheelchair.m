function A = A_leggedWheelchair(in1,in2)
%A_leggedWheelchair
%    A = A_leggedWheelchair(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    30-Nov-2023 15:15:37

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
t2 = cos(th2);
t3 = cos(th4);
t4 = cos(th5);
t5 = sin(th1);
t6 = sin(th3);
t7 = sin(th5);
t8 = l_AC.*m4;
t9 = l_A_m3.*m3;
t10 = l_B_m2.*m2;
t11 = phiU+th5;
t12 = th1+th2;
t13 = th3+th4;
t14 = th3+th5;
t15 = N.^2;
t16 = b.^2;
t17 = l_AC.^2;
t18 = l_A_m3.^2;
t19 = l_B_m2.^2;
t20 = l_C_m4.^2;
t21 = l_OA.^2;
t22 = l_OB.^2;
t23 = l_O_m1.^2;
t24 = m1.*2.0;
t25 = m2.*2.0;
t26 = m3.*2.0;
t27 = m4.*2.0;
t28 = Ir.*N;
t30 = -I1;
t31 = -I2;
t32 = -I3;
t33 = -I4;
t34 = -Ir;
t42 = -th5;
t82 = l_C_m4.*l_OA.*m4.*-2.0;
t29 = cos(t11);
t35 = cos(t14);
t36 = sin(t11);
t37 = sin(t12);
t38 = sin(t13);
t39 = sin(t14);
t40 = t13+th1;
t41 = t13+th5;
t43 = l_cb.*mb.*t4;
t46 = l_cb.*mb.*t7;
t48 = Ir.*t15;
t49 = l_AC.*t8;
t50 = l_A_m3.*t9;
t51 = l_B_m2.*t10;
t52 = m4.*t20;
t53 = m3.*t21;
t54 = m4.*t21;
t55 = m1.*t23;
t56 = l_C_m4.*l_OA.*t27;
t57 = -t28;
t58 = b.*t4.*t24;
t59 = b.*t4.*t25;
t60 = b.*t4.*t26;
t61 = b.*t4.*t27;
t62 = b.*t7.*t24;
t63 = b.*t7.*t25;
t64 = b.*t7.*t26;
t65 = b.*t7.*t27;
t66 = l_C_m4.*t2.*t8;
t67 = l_C_m4.*t3.*t8;
t68 = l_OA.*t2.*t8;
t69 = l_OA.*t3.*t8;
t70 = l_OA.*t2.*t9;
t71 = l_OA.*t3.*t9;
t72 = l_OB.*t2.*t10;
t73 = b.*l_C_m4.*m4.*t5;
t74 = b.*l_C_m4.*m4.*t6;
t75 = b.*l_OA.*m3.*t5;
t76 = b.*l_OA.*m4.*t5;
t77 = b.*l_OA.*m3.*t6;
t78 = b.*l_OA.*m4.*t6;
t79 = b.*l_O_m1.*m1.*t5;
t80 = b.*l_O_m1.*m1.*t6;
t81 = t22.*t25;
t83 = t42+th1;
t85 = b.*m1.*t7.*-2.0;
t86 = b.*m2.*t7.*-2.0;
t87 = b.*m3.*t7.*-2.0;
t88 = b.*m4.*t7.*-2.0;
t90 = t12+t42;
t99 = b.*l_OB.*t5.*t25;
t106 = m2.*t22.*-2.0;
t142 = t8+t9+t10;
t178 = mU+ma+mb+t24+t25+t26+t27;
t44 = cos(t40);
t45 = cos(t41);
t47 = sin(t41);
t84 = cos(t83);
t89 = sin(t83);
t91 = -t46;
t92 = t66.*2.0;
t93 = t67.*2.0;
t94 = t68.*2.0;
t95 = t69.*2.0;
t96 = t70.*2.0;
t97 = t71.*2.0;
t98 = t72.*2.0;
t100 = -t49;
t101 = -t50;
t102 = -t51;
t103 = -t52;
t104 = -t53;
t105 = -t54;
t107 = -t55;
t108 = lU.*mU.*t29;
t109 = l_C_m4.*m4.*t35;
t110 = l_OA.*m3.*t35;
t111 = l_OA.*m4.*t35;
t112 = l_O_m1.*m1.*t35;
t113 = lU.*mU.*t36;
t114 = l_C_m4.*m4.*t39;
t115 = l_OA.*m3.*t39;
t116 = l_OA.*m4.*t39;
t117 = l_O_m1.*m1.*t39;
t118 = b.*t8.*t37;
t119 = b.*t8.*t38;
t120 = b.*t9.*t37;
t121 = b.*t9.*t38;
t122 = b.*t10.*t37;
t123 = b.*t10.*t38;
t130 = -t66;
t132 = -t68;
t134 = -t70;
t136 = -t72;
t138 = cos(t90);
t139 = sin(t90);
t182 = I2+I3+t48+t49+t50+t51;
t183 = I2+I3+t28+t49+t50+t51+t67+t69+t71;
t184 = I2+I3+t28+t49+t50+t51+t66+t68+t70+t72;
t124 = t8.*t45;
t125 = t9.*t45;
t126 = t10.*t45;
t127 = t8.*t47;
t128 = t9.*t47;
t129 = t10.*t47;
t131 = -t92;
t133 = -t94;
t135 = -t96;
t137 = -t98;
t140 = l_OB.*t10.*t44;
t141 = -t113;
t143 = l_C_m4.*m4.*t84;
t144 = l_OA.*m3.*t84;
t145 = l_OA.*m4.*t84;
t146 = l_O_m1.*m1.*t84;
t147 = l_C_m4.*m4.*t89;
t148 = l_OA.*m3.*t89;
t149 = l_OA.*m4.*t89;
t150 = l_O_m1.*m1.*t89;
t151 = l_OB.*t25.*t84;
t152 = l_OB.*t25.*t89;
t153 = t8.*t138;
t154 = t9.*t138;
t155 = t10.*t138;
t157 = t8.*t139;
t158 = t9.*t139;
t159 = t10.*t139;
t163 = l_OB.*m2.*t84.*-2.0;
t168 = l_OB.*m2.*t89.*-2.0;
t176 = t45.*t142;
t177 = t47.*t142;
t179 = t138.*t142;
t180 = t139.*t142;
t189 = t31+t32+t57+t100+t101+t102+t118+t120+t122+t130+t132+t134+t136;
t156 = -t140;
t160 = -t143;
t161 = -t144;
t162 = -t145;
t164 = -t146;
t165 = -t147;
t166 = -t148;
t167 = -t149;
t169 = -t150;
t170 = -t153;
t171 = -t154;
t172 = -t155;
t173 = -t157;
t174 = -t158;
t175 = -t159;
t181 = -t179;
t185 = t114+t115+t116+t117+t127+t128+t129;
t186 = t109+t110+t111+t112+t124+t125+t126;
t187 = t119+t121+t123+t140+t183;
t188 = t147+t148+t149+t150+t152+t157+t158+t159;
t191 = I1+I2+I3+I4+Ir+t28+t49+t50+t51+t52+t53+t54+t55+t56+t74+t77+t78+t80+t93+t95+t97+t119+t121+t123+t140;
t190 = t160+t161+t162+t163+t164+t170+t171+t172;
t192 = t85+t86+t87+t88+t91+t141+t143+t144+t145+t146+t151+t153+t154+t155+t186;
t193 = t30+t31+t32+t33+t34+t57+t73+t75+t76+t79+t82+t99+t100+t101+t102+t103+t104+t105+t106+t107+t118+t120+t122+t131+t133+t135+t137+t156;
t194 = t43+t58+t59+t60+t61+t108+t165+t166+t167+t168+t169+t173+t174+t175+t185;
mt1 = [I1+I4+Ir+t52+t53+t54+t55+t56+t81+t92+t94+t96+t98+t182,t184,t156,t156,t193,t190,t188,0.0,t184,t182,0.0,0.0,t189,t181,t180,0.0,t156,0.0,I1+I4+Ir+t52+t53+t54+t55+t56+t93+t95+t97+t182,t183,t191,t186,t185,0.0,t156,0.0,t183,t182,t187,t176,t177,0.0,t193,t189,t191,t187];
mt2 = [I1.*2.0+I2.*2.0+I3.*2.0+I4.*2.0+I_B+I_U+Ir.*4.0+t49.*2.0+t50.*2.0+t51.*2.0-t73.*2.0-t75.*2.0-t76.*2.0-t79.*2.0+t81+t92+t93+t94+t95+t96+t97+t98-t118.*2.0+t119.*2.0-t120.*2.0+t121.*2.0-t122.*2.0+t123.*2.0+t140.*2.0+t16.*t24+t16.*t25+t16.*t26+t16.*t27+t20.*t27+t21.*t26+t23.*t24+t21.*t27+lU.^2.*mU+l_cb.^2.*mb+l_C_m4.*l_OA.*m4.*4.0-b.*l_OB.*m2.*t5.*4.0+b.*l_C_m4.*t6.*t27+b.*l_OA.*t6.*t26+b.*l_OA.*t6.*t27+b.*l_O_m1.*t6.*t24,t192,t194,0.0,t190,t181,t186,t176,t192,t178,0.0,0.0,t188,t180,t185,t177,t194,0.0,t178,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,I_A];
A = reshape([mt1,mt2],8,8);
