package us.ihmc.tMotorCore.parameters;

public interface TMotorParameters
{
    float getPositionLimitLower();
    float getPositionLimitUpper();

    float getVelocityLimitLower();
    float getVelocityLimitUpper();

    float getTorqueLimitLower();
    float getTorqueLimitUpper();

    float getMaximumKp();
    float getMaximumKd();

    double getGearRatio();
}
