function p = manipulator_parameters()
%MANIPULATOR_PARAMETERS Parameters shared with the Arduino arm controller.
%
% Coordinate convention
%   X-Y : horizontal plane
%   Z   : upward
%   q1  : base yaw about +Z
%   q2  : shoulder pitch from the horizontal plane
%   q3  : elbow relative pitch
%   q4  : wrist pitch; toolPitch = q2 + q3 + q4
%   q5  : wrist roll (not required for position IK)
%   q6  : gripper opening (not required for position IK)

p.name = "DS51150 Hexapod Manipulator";

% The Arduino controller currently uses 300 mm for both arm links.
% Replace these values after measuring the rotation-axis distances in CAD.
p.link1Mm = 300;
p.link2Mm = 300;
p.toolOffsetMm = 115;
p.baseHeightMm = 90;

% J1~J4 use DS51150-270. J5~J6 use SPT5435LV-180.
p.servoName = ["DS51150-270", "DS51150-270", "DS51150-270", ...
               "DS51150-270", "SPT5435LV-180", "SPT5435LV-180"];
p.homeUs = [1290, 1600, 1480, 1510, 1610, 1540];
p.servoDirection = [1, 1, -1, 1, 1, 1];
p.usPerDegree = [2000/270, 2000/270, 2000/270, 2000/270, ...
                 2000/180, 2000/180];
p.minPulseUs = [600, 600, 600, 600, 800, 800];
p.maxPulseUs = [2400, 2400, 2400, 2400, 2200, 2200];

p.jointMinDeg = [-90, -10, -125, -120, -90, 0];
p.jointMaxDeg = [ 90,  80,  -20,   60,  90, 45];
p.readyDeg = [0, 35, -100, -25, 0, 0];

% Demonstration trajectory settings.
p.sampleTimeSec = 0.04;
p.durationSec = 8;
p.desiredToolPitchDeg = 0;
end
