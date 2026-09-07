function [qDeg, info] = manipulator_ik(targetMm, toolPitchDeg, p)
%MANIPULATOR_IK Analytic IK for base yaw and a planar three-link arm.
%
% The position is solved with J1~J3. J4 maintains the requested tool pitch.
% The negative-acos branch is selected to match the folded elbow convention
% used by the Arduino controller.

arguments
    targetMm (1,3) double
    toolPitchDeg (1,1) double
    p (1,1) struct
end

x = targetMm(1);
y = targetMm(2);
z = targetMm(3);
phi = deg2rad(toolPitchDeg);

q1 = atan2(y, x);
radial = hypot(x, y);

% Remove the known tool offset and solve the two-link wrist-center problem.
wristR = radial - p.toolOffsetMm*cos(phi);
wristZ = z - p.baseHeightMm - p.toolOffsetMm*sin(phi);

cosQ3Raw = (wristR^2 + wristZ^2 - p.link1Mm^2 - p.link2Mm^2) / ...
           (2*p.link1Mm*p.link2Mm);
reachable = abs(cosQ3Raw) <= 1 + 1e-10;
cosQ3 = min(1, max(-1, cosQ3Raw));

q3 = -acos(cosQ3);
q2 = atan2(wristZ, wristR) - ...
     atan2(p.link2Mm*sin(q3), p.link1Mm + p.link2Mm*cos(q3));
q4 = phi - q2 - q3;

qDeg = [rad2deg(q1), rad2deg(q2), rad2deg(q3), rad2deg(q4), 0, 0];
withinLimits = all(qDeg >= p.jointMinDeg - 1e-9 & ...
                   qDeg <= p.jointMaxDeg + 1e-9);

info.reachable = reachable;
info.withinLimits = withinLimits;
info.valid = reachable && withinLimits;
info.wristCenterMm = [wristR, wristZ];
info.cosQ3Raw = cosQ3Raw;
end
