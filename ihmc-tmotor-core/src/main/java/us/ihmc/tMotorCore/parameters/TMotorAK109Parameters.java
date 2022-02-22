package us.ihmc.tMotorCore.parameters;

public class TMotorAK109Parameters implements TMotorParameters
{
   private static final double MIN_POSITION = -12.5f;
   private static final double MAX_POSITION = 12.5f;
   private static final double MIN_VELOCITY = -46.57f;  // -23.24 for 24V and -46.57 for 48 V
   private static final double MAX_VELOCITY = 46.57f;   // 23.24 for 24V and 46.57 for 48 V
   private static final double MIN_TORQUE = -16.0f;
   private static final double MAX_TORQUE = 16.0f;

   private final static double MAXIMUM_KP = 500;
   private final static double MAXIMUM_KD = 100;

   public double getPositionLimitLower()
   {
      return MIN_POSITION;
   }

   public double getPositionLimitUpper()
   {
      return MAX_POSITION;
   }

   public double getVelocityLimitLower()
   {
      return MIN_VELOCITY;
   }

   public double getVelocityLimitUpper()
   {
      return MAX_VELOCITY;
   }

   public double getTorqueLimitLower()
   {
      return MIN_TORQUE;
   }

   public double getTorqueLimitUpper()
   {
      return MAX_TORQUE;
   }

   public double getMaximumKp()
   {
      return MAXIMUM_KP;
   }

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
   public double getMotorResistance()
   {
      return 0.09;
   }

   /**
    * The temperature coefficient of resistance (0.00393 for copper):
    */
   @Override
   public double getHeatingCoefficient()
   {
      return 0.0135;
   }

   /**
    * The thermal resistance to ambient 
    */
   @Override
   public double getCoolingCoefficient()
   {
      return 0.0034;
   }

   @Override
   public double getKt()
   {
      return 0.095;
   }
}
