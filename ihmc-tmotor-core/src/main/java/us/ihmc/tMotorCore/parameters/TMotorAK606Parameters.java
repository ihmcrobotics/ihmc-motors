package us.ihmc.tMotorCore.parameters;

public class TMotorAK606Parameters implements TMotorParameters
{
   private static final double MIN_POSITION = -12.5f;
   private static final double MAX_POSITION = 12.5f;
   private static final double MIN_VELOCITY = -41.87f;
   private static final double MAX_VELOCITY = 41.87f;
   private static final double MIN_TORQUE = -9.0f;
   private static final double MAX_TORQUE = 9.0f;

   /** Peak torque according to <a href="https://store.tmotor.com/h5/#/product?id=1201">Specs</a> */
   private static final double PEAK_TORQUE = 9.0;

   private final static double MAXIMUM_KP = 500;
   private final static double MAXIMUM_KD = 100;

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
      return 6.0;
   }

   @Override
   public double getKt()
   {
      return 0.113;
   }

   @Override
   public double getCoilThermalMass()
   {
      return 17.70070787; // fitted. ask evan
   }

   @Override
   public double getMotorThermalMass()
   {
      return 18.81621937; // fitted. ask evan
   }

   @Override
   public double getEnvironmentThermalMass()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getMotorCoilConductivity()
   {
      return 0.26140927; // fitted. ask evan
   }

   @Override
   public double getEnvMotorConductivity()
   {
      return 0.02798457; // fitted. ask evan
   }

   @Override
   public double getCurrentAlpha()
   {
      return 0.00393;
   }

   @Override
   public double getElectricalResistance()
   {
      return 0.262; // https://store.tmotor.com/goods.php?id=1138
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
