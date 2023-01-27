package us.ihmc.tMotorCore;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

// detects the first position jump and holds the value before the jump until reset
public class TMotorPositionJumpDetector
{
   // This assumes that each tick is 0.001 s
   private final double qChangeThresholdForJump = 0.6; // rads/tick
   private final YoBoolean positionJumpHasOccurred;
   private final YoDouble lastPositionBeforeJump;
   private final DoubleProvider positionProvider;

   public TMotorPositionJumpDetector(String prefix, DoubleProvider positionProvider, YoRegistry registry)
   {
      positionJumpHasOccurred  = new YoBoolean(prefix + "ActuatorPositionJumpHasOccurred", registry);

      lastPositionBeforeJump = new YoDouble(prefix + "LastActuatorPositionBeforeJump", registry);
      this.positionProvider = positionProvider;
   }

   public void update()
   {
      double currentPosition = positionProvider.getValue();
      double positionChangeThisTick = Math.abs(currentPosition - lastPositionBeforeJump.getDoubleValue());
      if (positionChangeThisTick > qChangeThresholdForJump)
      {
         positionJumpHasOccurred.set(true);
      }

      if (!positionJumpHasOccurred.getBooleanValue())
      {
         lastPositionBeforeJump.set(currentPosition);
      }

   }

   public boolean getPositionJumpHasOccurred()
   {
      return positionJumpHasOccurred.getBooleanValue();
   }

   public double getLastPositionBeforeJump()
   {
      return lastPositionBeforeJump.getDoubleValue();
   }

   public void resetDetector()
   {
      positionJumpHasOccurred.set(false);
   }
}
