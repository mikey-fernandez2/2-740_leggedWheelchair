function rHip = position_hip(in1,in2)
%POSITION_HIP
%    rHip = POSITION_HIP(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    01-Dec-2023 15:01:42

b = in2(17,:);
th5 = in1(5,:);
x = in1(6,:);
y = in1(7,:);
rHip = [x+b.*cos(th5);y+b.*sin(th5)];
