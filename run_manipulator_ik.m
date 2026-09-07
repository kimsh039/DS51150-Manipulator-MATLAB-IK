function results = run_manipulator_ik(varargin)
%RUN_MANIPULATOR_IK Run and document the manipulator IK simulation.
%
%   run_manipulator_ik
%   run_manipulator_ik("Animate", false)
%   results = run_manipulator_ik("SaveResults", true)

parser = inputParser;
addParameter(parser, "Animate", true, @(x)islogical(x) && isscalar(x));
addParameter(parser, "SaveResults", true, @(x)islogical(x) && isscalar(x));
parse(parser, varargin{:});
options = parser.Results;

p = manipulator_parameters();
rootDir = fileparts(mfilename("fullpath"));
resultsDir = fullfile(rootDir, "results");
if options.SaveResults && ~isfolder(resultsDir)
    mkdir(resultsDir);
end

t = (0:p.sampleTimeSec:p.durationSec).';
phase = 2*pi*t/p.durationSec;

% Smooth 3-D closed path that remains inside the configured joint limits.
radialMm = 435 + 35*cos(phase);
azimuthRad = deg2rad(18*sin(phase));
targetMm = [radialMm.*cos(azimuthRad), ...
            radialMm.*sin(azimuthRad), ...
            190 + 20*sin(phase)];

sampleCount = numel(t);
qDeg = zeros(sampleCount, 6);
achievedMm = zeros(sampleCount, 3);
valid = false(sampleCount, 1);
positionErrorMm = zeros(sampleCount, 1);

for k = 1:sampleCount
    [qDeg(k,:), info] = manipulator_ik( ...
        targetMm(k,:), p.desiredToolPitchDeg, p);
    valid(k) = info.valid;
    [achievedMm(k,:), ~] = manipulator_fk(qDeg(k,:), p);
    positionErrorMm(k) = norm(achievedMm(k,:) - targetMm(k,:));
end

if ~all(valid)
    badSample = find(~valid, 1, "first");
    error("ManipulatorIK:InvalidTrajectory", ...
          "Target sample %d is unreachable or violates a joint limit.", badSample);
end

pulseUs = joint_angles_to_pwm(qDeg, p);

results = table(t, targetMm(:,1), targetMm(:,2), targetMm(:,3), ...
    achievedMm(:,1), achievedMm(:,2), achievedMm(:,3), ...
    positionErrorMm, qDeg(:,1), qDeg(:,2), qDeg(:,3), qDeg(:,4), ...
    qDeg(:,5), qDeg(:,6), pulseUs(:,1), pulseUs(:,2), pulseUs(:,3), ...
    pulseUs(:,4), pulseUs(:,5), pulseUs(:,6), ...
    'VariableNames', {'time_s', 'target_x_mm', 'target_y_mm', 'target_z_mm', ...
    'achieved_x_mm', 'achieved_y_mm', 'achieved_z_mm', 'position_error_mm', ...
    'J1_deg', 'J2_deg', 'J3_deg', 'J4_deg', 'J5_deg', 'J6_deg', ...
    'J1_us', 'J2_us', 'J3_us', 'J4_us', 'J5_us', 'J6_us'});

visibility = "off";
if options.Animate
    visibility = "on";
end

armFigure = figure("Name", "Manipulator IK animation", ...
                   "Color", "white", "Visible", visibility);
ax = axes(armFigure);
frameStep = max(1, round(0.08/p.sampleTimeSec));
if options.Animate
    for k = 1:frameStep:sampleCount
        [~, pointsMm] = manipulator_fk(qDeg(k,:), p);
        draw_manipulator(ax, pointsMm, targetMm(k,:), targetMm, ...
                         achievedMm(1:k,:), p);
        subtitle(ax, sprintf("t = %.2f s | error = %.3g mm", ...
                             t(k), positionErrorMm(k)));
        drawnow;
    end
end
[~, finalPointsMm] = manipulator_fk(qDeg(end,:), p);
draw_manipulator(ax, finalPointsMm, targetMm(end,:), targetMm, achievedMm, p);
subtitle(ax, sprintf("Maximum FK/IK residual = %.3g mm", max(positionErrorMm)));

jointFigure = figure("Name", "Manipulator joint angles", ...
                     "Color", "white", "Visible", visibility);
tiledlayout(jointFigure, 2, 1, "TileSpacing", "compact");
jointAx = nexttile;
plot(jointAx, t, qDeg(:,1:4), "LineWidth", 1.5);
set(jointAx, "Color", "white", "XColor", [0.15 0.15 0.15], ...
    "YColor", [0.15 0.15 0.15], "GridColor", [0.75 0.75 0.75]);
grid(jointAx, "on");
ylabel(jointAx, "Angle (deg)");
title(jointAx, "IK joint trajectory", "Color", [0.10 0.10 0.10]);
legend(jointAx, "J1 base", "J2 shoulder", "J3 elbow", "J4 wrist pitch", ...
       "Location", "eastoutside", "Color", "white", ...
       "TextColor", [0.10 0.10 0.10], "EdgeColor", [0.60 0.60 0.60]);

errorAx = nexttile;
semilogy(errorAx, t, max(positionErrorMm, eps), "LineWidth", 1.5, ...
         "Color", [0.85 0.25 0.20]);
set(errorAx, "Color", "white", "XColor", [0.15 0.15 0.15], ...
    "YColor", [0.15 0.15 0.15], "GridColor", [0.75 0.75 0.75], ...
    "MinorGridColor", [0.88 0.88 0.88]);
grid(errorAx, "on");
xlabel(errorAx, "Time (s)");
ylabel(errorAx, "Position residual (mm)");
title(errorAx, "Forward/inverse kinematics consistency", ...
      "Color", [0.10 0.10 0.10]);

if options.SaveResults
    exportgraphics(armFigure, fullfile(resultsDir, "ik_trajectory.png"), ...
                   "Resolution", 180);
    exportgraphics(jointFigure, fullfile(resultsDir, "joint_angles_and_error.png"), ...
                   "Resolution", 180);
    writetable(results, fullfile(resultsDir, "ik_results.csv"));
end

fprintf("Manipulator IK simulation complete.\n");
fprintf("  Samples: %d\n", sampleCount);
fprintf("  Reachable and within limits: %d/%d\n", nnz(valid), sampleCount);
fprintf("  Maximum position residual: %.6g mm\n", max(positionErrorMm));
fprintf("  J1 range: %.2f to %.2f deg\n", min(qDeg(:,1)), max(qDeg(:,1)));
fprintf("  J2 range: %.2f to %.2f deg\n", min(qDeg(:,2)), max(qDeg(:,2)));
fprintf("  J3 range: %.2f to %.2f deg\n", min(qDeg(:,3)), max(qDeg(:,3)));
fprintf("  J4 range: %.2f to %.2f deg\n", min(qDeg(:,4)), max(qDeg(:,4)));

if ~options.Animate
    close(armFigure);
    close(jointFigure);
end
end
