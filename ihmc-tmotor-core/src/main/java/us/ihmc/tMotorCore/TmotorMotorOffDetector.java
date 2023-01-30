package us.ihmc.tMotorCore;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class TmotorMotorOffDetector
{
   private final int repeatedMeasurementThresholdForOff = 100;
   private int numRepeatedMeasurements = 0;
   private double lastTorque = 0;
   private final DoubleProvider torqueSupplier;
   private final YoBoolean motorIsOff;
   private final int ticksPerUpdate;
   private int updateTick = 0;

   private final YoInteger repeatedMeasurements;
   private final YoDouble yoLastTorque;
   private final YoDouble yoCurrentTorque;
   private final YoDouble dTau;

   public TmotorMotorOffDetector(String prefix, DoubleProvider torqueSupplier, int ticksPerUpdate, YoRegistry registry)
   {
      this.torqueSupplier = torqueSupplier;
      motorIsOff = new YoBoolean(prefix + "ActuatorIsOff", registry);
      motorIsOff.set(false);

      this.ticksPerUpdate = ticksPerUpdate;

      repeatedMeasurements = new YoInteger(prefix + "DEBUG_repeated_measurements", registry);
      yoLastTorque = new YoDouble(prefix + "DEBUG_lastTorque", registry);
      yoCurrentTorque = new YoDouble(prefix + "DEBUG_currentTorque", registry);
      dTau = new YoDouble(prefix + "DEBUG_dTau", registry);
   }

   // CAN message reads are at 500 hz...2 ticks per read
   public void update()
   {
      if (updateTick == ticksPerUpdate)
      {
         // Check if torque reading is changing
         double currentTorque = torqueSupplier.getValue();

         dTau.set(currentTorque - lastTorque);
         yoCurrentTorque.set(currentTorque);
         yoLastTorque.set(lastTorque);

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

         // If value isn't changing enough times, means that motor is off
         if (numRepeatedMeasurements >= repeatedMeasurementThresholdForOff)
         {
            motorIsOff.set(true);
         }

         updateTick = 0;
      }

      updateTick += 1;
   }

   public boolean getMotorIsOff()
   {
      return motorIsOff.getValue();
   }
}
