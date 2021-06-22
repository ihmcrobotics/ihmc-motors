package us.ihmc.sensors;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TorqueToForceTransmission
{
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

    private static double MOTOR_PULLEY_RADIUS;      // in meters
    private final YoDouble desiredForce = new YoDouble("desiredForce", registry);

    public TorqueToForceTransmission(double motorPulleyRadius, YoRegistry parentRegistry)
    {
        MOTOR_PULLEY_RADIUS = motorPulleyRadius;
        parentRegistry.addChild(registry);
    }

    public void update(double torque)
    {
        desiredForce.set(torque / MOTOR_PULLEY_RADIUS);
    }
}
