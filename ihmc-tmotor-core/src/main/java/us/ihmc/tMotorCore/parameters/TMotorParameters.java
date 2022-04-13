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

   double getKt();

   double getCoilThermalMass();

   double getMotorThermalMass();

   double getEnvironmentThermalMass();

   double getMotorCoilConductivity();

   double getEnvMotorConductivity();

   double getCurrentAlpha();

   double getElectricalResistance();

   double getAmbientResistorTemperature();

   double getDefaultAmbientTemperature();
}
