function f = stateFcnBase(in1,in2)
%stateFcnBase
%    F = stateFcnBase(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    06-Oct-2024 18:52:48

omega = in2(2,:);
phi = in1(1,:);
v = in2(1,:);
f = [omega;v.*cos(phi);v.*sin(phi)];
end
