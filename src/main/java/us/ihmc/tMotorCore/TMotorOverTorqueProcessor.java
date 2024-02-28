package us.ihmc.tMotorCore;

import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.function.BooleanSupplier;

public class TMotorOverTorqueProcessor
{
   private static final double TORQUE_THRESHOLD_NEAR_MAX = 0.3;
   private static final int GLITCH_FILTER_WINDOW_SIZE = 15;
   private static final int MIN_MAX_OFFSET = 1;
   private final YoInteger torqueOffset;
   private final GlitchFilteredYoBoolean overTorqueDetected;
   private final double nominalTorqueLimit;
   private final DoubleProvider torqueScale;
   private final DoubleProvider measuredTorque;
   private boolean firstTick = true;
   private double previousTrustedMeasuredTorque;
   private BooleanSupplier enabled = () -> true;

   public TMotorOverTorqueProcessor(String prefix,
                                    YoDouble yoTime,
                                    double nominalTorqueLimit,
                                    DoubleProvider torqueScale,
                                    DoubleProvider measuredTorque,
                                    YoRegistry registry)
   {
      this.nominalTorqueLimit = nominalTorqueLimit;
      this.measuredTorque = measuredTorque;
      this.torqueScale = torqueScale;
      this.torqueOffset = new YoInteger(prefix + "TorqueOffset", registry);
      this.overTorqueDetected = new GlitchFilteredYoBoolean(prefix + "OverTorqueDetected", registry, GLITCH_FILTER_WINDOW_SIZE);
   }

   public void initialize()
   {
      torqueOffset.set(0);
      previousTrustedMeasuredTorque = measuredTorque.getValue();
      overTorqueDetected.set(false);
   }

   /**
    * Returns torque that has been compensated for wrap-around
    */
   public double computeWrapAroundCompensatedTorque()
   {
      if (!enabled.getAsBoolean())
      {
         // Assumes no wrap around after shaking is done. Could add more band-aids if these aren't enough
         initialize();
         return measuredTorque.getValue();
      }

      if (firstTick)
      {
         initialize();
         firstTick = false;
      }

      return computeInternal();
   }

   private double computeInternal()
   {
      double minMaxTorque = nominalTorqueLimit * torqueScale.getValue();
      double minMaxTorqueThreshold = (1.0 - TORQUE_THRESHOLD_NEAR_MAX) * minMaxTorque;

      boolean negativeWrapAround = previousTrustedMeasuredTorque < -minMaxTorqueThreshold && measuredTorque.getValue() > minMaxTorqueThreshold;
      boolean positiveWrapAround = previousTrustedMeasuredTorque > minMaxTorqueThreshold && measuredTorque.getValue() < -minMaxTorqueThreshold;
      overTorqueDetected.update(negativeWrapAround || positiveWrapAround);

      if (overTorqueDetected.getValue())
      {
         if (negativeWrapAround)
         {
            torqueOffset.set(Math.max(-MIN_MAX_OFFSET, torqueOffset.getValue() - 1));
         }
         if (positiveWrapAround)
         {
            torqueOffset.set(Math.min(MIN_MAX_OFFSET, torqueOffset.getValue() + 1));
         }

         overTorqueDetected.set(false);
         previousTrustedMeasuredTorque = measuredTorque.getValue();
      }
      else if (negativeWrapAround)
      {
         return -minMaxTorque + 2.0 * torqueOffset.getValue() * nominalTorqueLimit * torqueScale.getValue();
      }
      else if (positiveWrapAround)
      {
         return minMaxTorque + 2.0 * torqueOffset.getValue() * nominalTorqueLimit * torqueScale.getValue();
      }

      if (Math.abs(measuredTorque.getValue() - previousTrustedMeasuredTorque) < 0.5 * minMaxTorque)
      {
         previousTrustedMeasuredTorque = measuredTorque.getValue();
      }

      return measuredTorque.getValue() + 2.0 * torqueOffset.getValue() * nominalTorqueLimit * torqueScale.getValue();
   }

   public void setEnabled(BooleanSupplier enabled)
   {
      this.enabled = enabled;
   }
}