package us.ihmc.tMotorCore.CANMessages;

import peak.can.basic.TPCANMsg;
import us.ihmc.CAN.CANTools;
import us.ihmc.tMotorCore.parameters.TMotorParameters;

public class TMotorCANReplyMessage
{
    private final float MIN_POSITION;
    private final float MAX_POSITION;
    private final float MIN_VELOCITY;
    private final float MAX_VELOCITY;
    private final float MIN_TORQUE;
    private final float MAX_TORQUE;

    private int measuredEncoderPosition;

    private float measuredPosition;
    private float measuredVelocity;
    private float measuredTorque;

    public TMotorCANReplyMessage(TMotorParameters encoderParameters)
    {
        MIN_POSITION = encoderParameters.getMinimumEncoderPosition();
        MAX_POSITION = encoderParameters.getMaximumEncoderPosition();
        MIN_VELOCITY = encoderParameters.getMinimumEncoderVelocity();
        MAX_VELOCITY = encoderParameters.getMaximumEncoderVelocity();
        MIN_TORQUE = encoderParameters.getMinimumTorqueReading();
        MAX_TORQUE = encoderParameters.getMaximumTorqueReading();
    }

    public void parseAndUnpack(TPCANMsg message)
    {
        int Data1 = Byte.toUnsignedInt(message.getData()[1]);
        int Data2 = Byte.toUnsignedInt(message.getData()[2]);
        int Data3 = Byte.toUnsignedInt(message.getData()[3]);
        int Data4 = Byte.toUnsignedInt(message.getData()[4]);
        int Data5 = Byte.toUnsignedInt(message.getData()[5]);

        // parse message
        int id = message.getData()[0];
        measuredEncoderPosition = (Data1 << 8) | Data2;
        int measuredEncoderVelocity = (Data3 << 4) | (Data4 >> 4);
        int measuredEncoderTorque = ((Data4 & 0xF) << 8) | Data5;

        //
        // convert sensor values to meaningful (float) units
        //
        // position of output shaft
        measuredPosition = CANTools.uint_to_float(measuredEncoderPosition, MIN_POSITION, MAX_POSITION, 16);
        measuredVelocity = CANTools.uint_to_float(measuredEncoderVelocity, MIN_VELOCITY, MAX_VELOCITY, 12);
        measuredTorque = CANTools.uint_to_float(measuredEncoderTorque, MIN_TORQUE, MAX_TORQUE, 12);
    }

    public float getMeasuredPosition()
    {
        return measuredPosition;
    }

    public float getMeasuredVelocity()
    {
        return measuredVelocity;
    }

    public float getMeasuredTorque()
    {
        return measuredTorque;
    }

    public int getMeasuredEncoderPosition() { return measuredEncoderPosition; }

    public static int getID(TPCANMsg message)
    {
        return message.getData()[0];
    }

}
