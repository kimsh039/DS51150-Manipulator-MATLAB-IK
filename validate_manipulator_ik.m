function validate_manipulator_ik()
%VALIDATE_MANIPULATOR_IK Lightweight regression checks without a toolbox.

p = manipulator_parameters();

testJointDeg = [12, 65, -105, 40, 0, 0];
[targetMm, ~] = manipulator_fk(testJointDeg, p);
toolPitchDeg = sum(testJointDeg(2:4));
[solvedDeg, info] = manipulator_ik(targetMm, toolPitchDeg, p);
[reconstructedMm, ~] = manipulator_fk(solvedDeg, p);

assert(info.valid, "Known reachable pose was rejected.");
assert(norm(reconstructedMm - targetMm) < 1e-8, ...
       "IK/FK reconstruction residual is too large.");
assert(max(abs(solvedDeg(1:4) - testJointDeg(1:4))) < 1e-8, ...
       "IK did not recover the expected elbow branch.");

results = run_manipulator_ik("Animate", false, "SaveResults", true);
assert(max(results.position_error_mm) < 1e-8, ...
       "Demonstration trajectory residual is too large.");

fprintf("All manipulator IK validation checks passed.\n");
end
