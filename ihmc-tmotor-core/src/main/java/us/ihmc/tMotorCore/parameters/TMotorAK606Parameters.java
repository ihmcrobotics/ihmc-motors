package us.ihmc.tMotorCore.parameters;

public class TMotorAK606Parameters implements TMotorParameters
{
   private static final double MIN_POSITION = -12.5f;
   private static final double MAX_POSITION = 12.5f;
   private static final double MIN_VELOCITY = -41.87f;
   private static final double MAX_VELOCITY = 41.87f;
   private static final double MIN_TORQUE = -9.0f;
   private static final double MAX_TORQUE = 9.0f;

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
        return 6.0;
    }

    /**
     * This is the phase to phase resistance in ohms
     */
   @Override
   public double getMotorResistance()
   {
      return 0.605;
   }

   /**
    * The temperature coefficient of resistance (0.00393 for copper):
    */
   @Override
   public double getHeatingCoefficient()
   {
      return 0.00393;
   }

   /**
    * The thermal resistance to ambient 
    */
   @Override
   public double getCoolingCoefficient()
   {
      return 1.0;
   }
   
   @Override
   public double getKt()
   {
      return 0.113;
   }
}
