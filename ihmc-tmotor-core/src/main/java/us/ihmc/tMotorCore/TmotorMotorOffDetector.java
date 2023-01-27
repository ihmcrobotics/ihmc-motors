package us.ihmc.tMotorCore;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class TmotorMotorOffDetector
{
   private final int repeatedMeasurementThresholdForOff = 4;
   private int numRepeatedMeasurements = 0;
   private final YoInteger repeatedMeasurements;
   private final YoDouble yoLastTorque;
   private double lastTorque = 0;
   private final DoubleProvider torqueSupplier;
   private final YoBoolean motorIsOff;

   public TmotorMotorOffDetector(String prefix, DoubleProvider torqueSupplier, YoRegistry registry)
   {
      this.torqueSupplier = torqueSupplier;
      motorIsOff = new YoBoolean(prefix + "ActuatorIsOff", registry);
      motorIsOff.set(false);
      repeatedMeasurements = new YoInteger(prefix + "DEBUG_repeated measurements", registry);
      yoLastTorque = new YoDouble(prefix + "DEBUG_lastTorque", registry);
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

      repeatedMeasurements.set(numRepeatedMeasurements);
      yoLastTorque.set(lastTorque);

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
