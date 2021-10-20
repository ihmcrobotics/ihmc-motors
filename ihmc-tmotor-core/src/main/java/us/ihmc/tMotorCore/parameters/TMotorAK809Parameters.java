package us.ihmc.tMotorCore.parameters;

public class TMotorAK809Parameters implements TMotorParameters
{
    private static final float MIN_POSITION = -12.5f;
    private static final float MAX_POSITION = 12.5f;
    private static final float MIN_VELOCITY = -41.87f;  // -25.64 according to data sheet
    private static final float MAX_VELOCITY = 41.87f;   // 25.64 according to data sheet
    private static final float MIN_TORQUE = -18.0f;     //TODO check if better with -9
    private static final float MAX_TORQUE = 18.0f;      //TODO check if better with 9

    private final static float MAXIMUM_KP = 500;
    private final static float MAXIMUM_KD = 100;

    private final static float TORQUE_RATIO = 0.46f;

    private final static int GEAR_RATIO_TO_ONE = 9;

    public TMotorAK809Parameters()
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
