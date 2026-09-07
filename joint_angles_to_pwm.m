function pulseUs = joint_angles_to_pwm(qDeg, p)
%JOINT_ANGLES_TO_PWM Convert simulated joint angles to calibrated PWM values.

arguments
    qDeg (:,6) double
    p (1,1) struct
end

pulseUs = p.homeUs + qDeg .* p.servoDirection .* p.usPerDegree;
pulseUs = min(p.maxPulseUs, max(p.minPulseUs, pulseUs));
end
