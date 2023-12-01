function rHip_feet = hip_relative_feet(in1,in2)
%HIP_RELATIVE_FEET
%    rHip_feet = HIP_RELATIVE_FEET(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    30-Nov-2023 15:15:39

l_AC = in2(15,:);
l_DE = in2(16,:);
l_OB = in2(14,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
t2 = l_DE+l_OB;
t3 = th3+th5;
t5 = -th5;
t4 = t3+th4;
t6 = t5+th1;
t7 = t6+th2;
rHip_feet = [l_AC.*sin(t7)+t2.*sin(t6);l_AC.*cos(t7)+t2.*cos(t6);-l_AC.*sin(t4)-t2.*sin(t3);l_AC.*cos(t4)+t2.*cos(t3)];
