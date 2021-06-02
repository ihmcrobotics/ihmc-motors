package us.ihmc.tMotorCore.parameters;

public class TMotorAK109Parameters implements TMotorParameters
{
    private static final float MIN_POSITION = -12.5f;
    private static final float MAX_POSITION = 12.5f;
    private static final float MIN_VELOCITY = -46.57f;  // -23.24 for 24V and -46.57 for 48 V
    private static final float MAX_VELOCITY = 46.57f;   // 23.24 for 24V and 46.57 for 48 V
    private static final float MIN_TORQUE = -54.0f;     //TODO check if better with -9
    private static final float MAX_TORQUE = 54.0f;      //TODO check

    private final static float MAXIMUM_KP = 500;
    private final static float MAXIMUM_KD = 100;

    private final static int GEAR_RATIO_TO_ONE = 9;

    public TMotorAK109Parameters()
    {

    }

    public float getMinimumEncoderPosition()
    {
        return MIN_POSITION;
    }


    public float getMaximumEncoderPosition()
    {
        return MAX_POSITION;
    }

    public float getMinimumEncoderVelocity()
    {
        return MIN_VELOCITY;
    }

    public float getMaximumEncoderVelocity()
    {
        return MAX_VELOCITY;
    }

    public float getMinimumTorqueReading()
    {
        return MIN_TORQUE;
    }

    public float getMaximumTorqueReading()
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

}
