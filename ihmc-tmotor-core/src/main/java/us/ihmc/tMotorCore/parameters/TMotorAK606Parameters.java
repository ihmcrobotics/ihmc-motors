package us.ihmc.tMotorCore.parameters;

public class TMotorAK606Parameters implements TMotorParameters
{
    private static final float MIN_POSITION = -12.5f;
    private static final float MAX_POSITION = 12.5f;
    private static final float MIN_VELOCITY = -41.87f;
    private static final float MAX_VELOCITY = 41.87f;
    private static final float MIN_TORQUE = -9.0f;
    private static final float MAX_TORQUE = 9.0f;

    private final static float MAXIMUM_KP = 500;
    private final static float MAXIMUM_KD = 100;

    private final static float TORQUE_RATIO = 1.0f;

    private final static int GEAR_RATIO_TO_ONE = 6;

    public TMotorAK606Parameters()
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

    @Override
    public float getTorqueRatio()
    {
        return TORQUE_RATIO;
    }
}
