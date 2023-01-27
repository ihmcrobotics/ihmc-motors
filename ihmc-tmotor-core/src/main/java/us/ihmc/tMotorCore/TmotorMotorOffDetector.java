package us.ihmc.tMotorCore;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class TmotorMotorOffDetector
{
   private final int repeatedMeasurementThresholdForOff = 4;
   private int numRepeatedMeasurements = 0;
   private double lastTorque = 0;
   private final DoubleProvider torqueSupplier;
   private final YoBoolean motorIsOff;

   public TmotorMotorOffDetector(String prefix, DoubleProvider torqueSupplier, YoRegistry registry)
   {
      this.torqueSupplier = torqueSupplier;
      motorIsOff = new YoBoolean(prefix + "ActuatorIsOff", registry);
      motorIsOff.set(false);
   }

   public void update()
   {
      // Check if torque reading is changing
      double currentTorque = torqueSupplier.getValue();
      if (currentTorque == lastTorque)
      {
         numRepeatedMeasurements += 1;
      }
      else
      {
         numRepeatedMeasurements = 0;
         lastTorque = currentTorque;
         motorIsOff.set(false);
      }

      // If value isn't changing enough times, means that motor is off
      if (numRepeatedMeasurements >= repeatedMeasurementThresholdForOff)
      {
         motorIsOff.set(true);
      }
   }

   public boolean getMotorIsOff()
   {
      return motorIsOff.getValue();
   }
}
