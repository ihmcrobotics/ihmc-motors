package us.ihmc.tMotorCore.parameters;

public class TMotorAK109Parameters implements TMotorParameters
{
   private static final double MIN_POSITION = -12.5f;
   private static final double MAX_POSITION = 12.5f;
   private static final double MIN_VELOCITY = -46.57f; // -23.24 for 24V and -46.57 for 48 V
   private static final double MAX_VELOCITY = 46.57f; // 23.24 for 24V and 46.57 for 48 V
   // TODO Not sure sure what these min/max torque values are supposed to represent, some doc would be useful here.
   private static final double MIN_TORQUE = -16.0f;
   private static final double MAX_TORQUE = 16.0f;
   /** Peak torque according to <a href="https://store.tmotor.com/h5/#/product?id=982">Specs</a> */
   private static final double PEAK_TORQUE = 38.0;

   private final static double MAXIMUM_KP = 500.0;
   private final static double MAXIMUM_KD = 100.0;

   @Override
   public double getPositionLimitLower()
   {
      return MIN_POSITION;
   }

   @Override
   public double getPositionLimitUpper()
   {
      return MAX_POSITION;
   }

   @Override
   public double getVelocityLimitLower()
   {
      return MIN_VELOCITY;
   }

   @Override
   public double getVelocityLimitUpper()
   {
      return MAX_VELOCITY;
   }

   @Override
   public double getTorqueLimitLower()
   {
      return MIN_TORQUE;
   }

   @Override
   public double getTorqueLimitUpper()
   {
      return MAX_TORQUE;
   }

   @Override
   public double getPeakTorque()
   {
      return PEAK_TORQUE;
   }

   @Override
   public double getMaximumKp()
   {
      return MAXIMUM_KP;
   }

   @Override
   public double getMaximumKd()
   {
      return MAXIMUM_KD;
   }

   @Override
   public double getGearRatio()
   {
      return 9.0;
   }

   @Override
   public double getKt()
   {
      return 0.095;
   }

   @Override
   public double getCoilThermalMass()
   {
      return 51.00502884; // These values were fitted in python. Ask Evan
   }

   @Override
   public double getMotorThermalMass()
   {
      return 246.16215547; // These values were fitted in python. Ask Evan
   }

   @Override
   public double getEnvironmentThermalMass()
   {
      return Double.POSITIVE_INFINITY; // huge
   }

   @Override
   public double getMotorCoilConductivity()
   {
      return 1.80814198; // These values were fitted in python. Ask Evan
   }

   @Override
   public double getEnvMotorConductivity()
   {
      return 0.37162564; // These values were fitted in python. Ask Evan
   }

   @Override
   public double getCurrentAlpha()
   {
      return 0.00393; // https://build-its-inprogress.blogspot.com/2019/
   }

   @Override
   public double getElectricalResistance()
   {
      return 0.09; // https://store.tmotor.com/goods.php?id=1148
   }

   @Override
   public double getAmbientResistorTemperature()
   {
      return 25.0;
   }

   @Override
   public double getDefaultAmbientTemperature()
   {
      return 25.0;
   }
}
