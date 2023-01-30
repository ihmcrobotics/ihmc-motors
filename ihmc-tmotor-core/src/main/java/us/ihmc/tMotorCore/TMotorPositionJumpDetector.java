package us.ihmc.tMotorCore;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

// detects the first position jump and holds the value before the jump until reset
public class TMotorPositionJumpDetector
{
   // This assumes that each tick is 0.002 s (CAN is at 500 Hz)
   private final double qChangeThresholdForJump = 0.4; // rads/tick
   private final YoBoolean positionJumpHasOccurred;
   private final YoDouble lastPositionBeforeJump;
   private final DoubleProvider positionProvider;
   private final int ticksPerUpdate;
   private int updateTick;

   private final YoDouble positionChange;

   public TMotorPositionJumpDetector(String prefix, DoubleProvider positionProvider, int ticksPerUpdate, YoRegistry registry)
   {
      positionJumpHasOccurred  = new YoBoolean(prefix + "ActuatorPositionJumpHasOccurred", registry);
      this.ticksPerUpdate = ticksPerUpdate;

      lastPositionBeforeJump = new YoDouble(prefix + "LastActuatorPositionBeforeJump", registry);
      this.positionProvider = positionProvider;
      positionJumpHasOccurred.set(false);

      positionChange = new YoDouble(prefix + "DEBUG_positionChange", registry);
   }

   public void update()
   {
      if (updateTick == ticksPerUpdate)
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

         positionChange.set(positionChangeThisTick);

         updateTick = 0;
      }

      updateTick += 1;
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
