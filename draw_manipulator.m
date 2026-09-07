function draw_manipulator(ax, pointsMm, targetMm, targetPathMm, achievedPathMm, p)
%DRAW_MANIPULATOR Draw a schematic 3-D arm and its Cartesian path.

cla(ax);
set(ax, "Color", "white", "XColor", [0.15 0.15 0.15], ...
    "YColor", [0.15 0.15 0.15], "ZColor", [0.15 0.15 0.15], ...
    "GridColor", [0.75 0.75 0.75], "MinorGridColor", [0.88 0.88 0.88]);
hold(ax, "on");

plot3(ax, achievedPathMm(:,1), achievedPathMm(:,2), achievedPathMm(:,3), ...
      "-", "Color", [0.10 0.40 0.80], "LineWidth", 2.0, ...
      "DisplayName", "IK path");
plot3(ax, targetPathMm(:,1), targetPathMm(:,2), targetPathMm(:,3), ...
      "--", "Color", [0.90 0.20 0.15], "LineWidth", 1.2, ...
      "Marker", "o", "MarkerIndices", 1:12:size(targetPathMm,1), ...
      "MarkerSize", 3, "DisplayName", "Target path");

segmentColors = [0.18 0.18 0.20; 0.10 0.10 0.12; 0.10 0.10 0.12; 0.55 0.18 0.65];
segmentWidths = [4, 7, 7, 4];
for k = 1:4
    plot3(ax, pointsMm(k:k+1,1), pointsMm(k:k+1,2), pointsMm(k:k+1,3), ...
          "-o", "Color", segmentColors(k,:), "LineWidth", segmentWidths(k), ...
          "MarkerSize", 6, "MarkerFaceColor", [0.95 0.35 0.20], ...
          "HandleVisibility", "off");
end

scatter3(ax, targetMm(1), targetMm(2), targetMm(3), 70, ...
         [0.95 0.15 0.10], "x", "LineWidth", 2, ...
         "DisplayName", "Current target");

axis(ax, "equal");
grid(ax, "on");
view(ax, 38, 24);
xlabel(ax, "X (mm)");
ylabel(ax, "Y (mm)");
zlabel(ax, "Z (mm)");
title(ax, p.name + " - analytic IK", "Color", [0.10 0.10 0.10]);
xlim(ax, [-100 650]);
ylim(ax, [-300 300]);
zlim(ax, [0 650]);
legend(ax, "Location", "northeast", "Color", "white", ...
       "TextColor", [0.10 0.10 0.10], "EdgeColor", [0.60 0.60 0.60]);
hold(ax, "off");
end
