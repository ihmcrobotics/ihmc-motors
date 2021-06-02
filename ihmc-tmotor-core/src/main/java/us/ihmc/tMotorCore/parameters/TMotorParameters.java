package us.ihmc.tMotorCore.parameters;

public interface TMotorParameters
{
    public abstract float getMinimumEncoderPosition();

    public abstract float getMaximumEncoderPosition();

    public abstract float getMinimumEncoderVelocity();

    public abstract float getMaximumEncoderVelocity();

    public abstract float getMinimumTorqueReading();

    public abstract float getMaximumTorqueReading();

    public abstract float getMaximumKp();

    public abstract float getMaximumKd();
}
