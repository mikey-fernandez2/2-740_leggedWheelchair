function drWheel = velocity_wheel(in1,in2)
%VELOCITY_WHEEL
%    drWheel = VELOCITY_WHEEL(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    08-Nov-2023 16:48:11

dx = in1(14,:);
dy = in1(15,:);
drWheel = [dx;dy];
end
