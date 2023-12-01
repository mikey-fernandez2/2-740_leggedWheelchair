function drHip = velocity_hip(in1,in2)
%VELOCITY_HIP
%    drHip = VELOCITY_HIP(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    30-Nov-2023 15:15:39

b = in2(17,:);
dth5 = in1(13,:);
dx = in1(14,:);
dy = in1(15,:);
th5 = in1(5,:);
drHip = [dx-b.*dth5.*sin(th5);dy+b.*dth5.*cos(th5)];
