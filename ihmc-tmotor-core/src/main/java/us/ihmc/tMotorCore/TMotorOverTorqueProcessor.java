package us.ihmc.tMotorCore;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class TMotorOverTorqueProcessor
{
   private static final double TORQUE_THRESHOLD_NEAR_MAX = 0.3;
   private static final double MIN_TORQUE_OFFSET_TIME = 1.0;

   private final YoDouble yoTime;
   private double previousOffsetUpdateTime;
   private final YoInteger torqueOffset;
   private final double nominalTorqueLimit;
   private final DoubleProvider torqueScale;
   private final DoubleProvider measuredTorque;
   private double previousRawMeasuredTorque;

   public TMotorOverTorqueProcessor(String prefix,
                                    YoDouble yoTime,
                                    double nominalTorqueLimit,
                                    DoubleProvider torqueScale,
                                    DoubleProvider measuredTorque,
                                    YoRegistry registry)
   {
      this.yoTime = yoTime;
      this.nominalTorqueLimit = nominalTorqueLimit;
      this.measuredTorque = measuredTorque;
      this.torqueScale = torqueScale;
      this.torqueOffset = new YoInteger(prefix + "TorqueOffset", registry);
   }

   public void initialize()
   {
      torqueOffset.set(0);
      previousOffsetUpdateTime = Double.NEGATIVE_INFINITY;
      previousRawMeasuredTorque = 0.0;
   }

   public double updateTorqueOffset()
   {
      if (yoTime.getValue() - previousOffsetUpdateTime > MIN_TORQUE_OFFSET_TIME)
         updateInternal();
      return torqueOffset.getValue() * nominalTorqueLimit * torqueScale.getValue();
   }

   private void updateInternal()
   {
      double minMaxTorque = nominalTorqueLimit * torqueScale.getValue();
      double minMaxTorqueThreshold = (1.0 - TORQUE_THRESHOLD_NEAR_MAX) * minMaxTorque;

      if (previousRawMeasuredTorque < -minMaxTorqueThreshold && measuredTorque.getValue() > minMaxTorqueThreshold)
      {
         torqueOffset.set(Math.min(1, torqueOffset.getValue() + 1));
         previousOffsetUpdateTime = yoTime.getDoubleValue();
      }
      else if (previousRawMeasuredTorque > minMaxTorqueThreshold && measuredTorque.getValue() < -minMaxTorqueThreshold)
      {
         torqueOffset.set(Math.max(-1, torqueOffset.getValue() - 1));
         previousOffsetUpdateTime = yoTime.getDoubleValue();
      }

      previousRawMeasuredTorque = measuredTorque.getValue();
   }
}
