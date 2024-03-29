package us.ihmc.sensors;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TorqueToForceTransmission
{
   private static double MOTOR_PULLEY_RADIUS;      // in meters
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble desiredForce;

   public TorqueToForceTransmission(double motorPulleyRadius, String prefix, YoRegistry parentRegistry)
   {
      MOTOR_PULLEY_RADIUS = motorPulleyRadius;
      desiredForce = new YoDouble(prefix + "desiredForce", registry);
      parentRegistry.addChild(registry);
   }

   public void update(double torque)
   {
      desiredForce.set(torque / MOTOR_PULLEY_RADIUS);
   }

   public double getDesiredForce()
   {
      return desiredForce.getDoubleValue();
   }

   public double getMotorPulleyRadius()
   {
      return MOTOR_PULLEY_RADIUS;
   }
}
