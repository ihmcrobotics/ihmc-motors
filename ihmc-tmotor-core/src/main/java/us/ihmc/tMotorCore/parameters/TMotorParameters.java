package us.ihmc.tMotorCore.parameters;

public interface TMotorParameters
{
   double getPositionLimitLower();

   double getPositionLimitUpper();

   double getVelocityLimitLower();

   double getVelocityLimitUpper();

   double getTorqueLimitLower();

   double getTorqueLimitUpper();

   double getMaximumKp();

   double getMaximumKd();

   double getGearRatio();

   double getMotorResistance();

   double getHeatingCoefficient();

   double getCoolingCoefficient();

   double getKt();
}
