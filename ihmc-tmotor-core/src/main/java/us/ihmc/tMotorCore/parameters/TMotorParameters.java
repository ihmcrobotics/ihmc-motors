package us.ihmc.tMotorCore.parameters;

public interface TMotorParameters
{
    float getMinimumEncoderPosition();
    float getMaximumEncoderPosition();

    float getMinimumEncoderVelocity();
    float getMaximumEncoderVelocity();

    float getMinimumTorqueReading();
    float getMaximumTorqueReading();

    float getMaximumKp();
    float getMaximumKd();

    double getGearRatio();
}
