function rWheel = position_wheel(in1,in2)
%POSITION_WHEEL
%    rWheel = POSITION_WHEEL(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    13-Nov-2023 13:28:09

r = in2(23,:);
x = in1(6,:);
y = in1(7,:);
rWheel = [x;-r+y];
