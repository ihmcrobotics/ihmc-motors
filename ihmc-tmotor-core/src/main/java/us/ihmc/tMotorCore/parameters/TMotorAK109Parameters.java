package us.ihmc.tMotorCore.parameters;

public class TMotorAK109Parameters implements TMotorParameters
{
   private static final float MIN_POSITION = -12.5f;
   private static final float MAX_POSITION = 12.5f;
   private static final float MIN_VELOCITY = -46.57f;  // -23.24 for 24V and -46.57 for 48 V
   private static final float MAX_VELOCITY = 46.57f;   // 23.24 for 24V and 46.57 for 48 V
   private static final float MIN_TORQUE = -16.0f;
   private static final float MAX_TORQUE = 16.0f;

   private final static float MAXIMUM_KP = 500;
   private final static float MAXIMUM_KD = 100;

   public float getPositionLimitLower()
   {
      return MIN_POSITION;
   }

   public float getPositionLimitUpper()
   {
      return MAX_POSITION;
   }

   public float getVelocityLimitLower()
   {
      return MIN_VELOCITY;
   }

   public float getVelocityLimitUpper()
   {
      return MAX_VELOCITY;
   }

   public float getTorqueLimitLower()
   {
      return MIN_TORQUE;
   }

   public float getTorqueLimitUpper()
   {
      return MAX_TORQUE;
   }

   public float getMaximumKp()
   {
      return MAXIMUM_KP;
   }

   public float getMaximumKd()
   {
      return MAXIMUM_KD;
   }

    @Override
    public double getGearRatio()
    {
        return 9.0;
    }
}
