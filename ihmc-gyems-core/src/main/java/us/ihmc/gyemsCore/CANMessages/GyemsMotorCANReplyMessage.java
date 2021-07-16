package us.ihmc.gyemsCore.CANMessages;

import peak.can.basic.TPCANMsg;
import us.ihmc.CAN.CANTools;
import us.ihmc.gyemsCore.parameters.GyemsMotorParameters;

public class GyemsMotorCANReplyMessage
{
    private int currentMotorTurnCount = 1;
    private int previousEncoderPositionTick = Integer.MIN_VALUE;

    private final int GEAR_RATIO_TO_ONE;
    private final int MAX_BIT_POSITION;
    private final int HALF_MAX_POSITION_BIT;
    private final int HALF_MAX_TORQUE_BIT;
    private final double MAX_CURRENT;

    private double measuredEncoderPosition;

    private double measuredActuatorPosition;
    private double measuredActuatorVelocity;
    private double measuredActuatorTorque;

    private int measuredTemperature;

    public GyemsMotorCANReplyMessage(GyemsMotorParameters encoderParameters)
    {
        GEAR_RATIO_TO_ONE = encoderParameters.getGearRatioToOne();
        MAX_BIT_POSITION = encoderParameters.getMaxPositionBit();
        HALF_MAX_POSITION_BIT = encoderParameters.getHalfMaxPositionBit();
        HALF_MAX_TORQUE_BIT = encoderParameters.getHalfMaxTorqueBit();
        MAX_CURRENT = encoderParameters.getMaxCurrent();
    }

    public void parseAndUnpack(TPCANMsg message)
    {
        int temperature = message.getData()[1];
        int currentLow = message.getData()[2];
        int currentHigh = message.getData()[3];
        int speedLow = message.getData()[4];
        int speedHigh = message.getData()[5];
        int positionLow = Byte.toUnsignedInt( message.getData()[6] );
        int positionHigh = Byte.toUnsignedInt( message.getData()[7] );

        // construct 16-bit values for position, speed, and torque
        int position = (positionHigh << 8) | positionLow;
        int speed = (speedHigh << 8) | speedLow;
        int current = (currentHigh << 8) | currentLow;

        //
        // convert sensor values to meaningful (float) units
        //
        // position of output shaft
        int encoderCountDifference = previousEncoderPositionTick - position;
        if((encoderCountDifference) > HALF_MAX_POSITION_BIT)
            currentMotorTurnCount++;
        else if(encoderCountDifference < -HALF_MAX_POSITION_BIT)
            currentMotorTurnCount--;

        double actuatorPosition = CANTools.motorToActuatorPosition(position, GEAR_RATIO_TO_ONE, MAX_BIT_POSITION, currentMotorTurnCount);

        // speed of output shaft
        double actuatorSpeed = CANTools.motorToActuatorSpeed(speed, GEAR_RATIO_TO_ONE);

        // torque current
        double torqueCurrent = current * MAX_CURRENT / HALF_MAX_TORQUE_BIT;  // from servo motor protocol documentation, this goes from -33A to 33A

        measuredTemperature = temperature;
        measuredEncoderPosition = position;
        measuredActuatorPosition = actuatorPosition;
        measuredActuatorVelocity = actuatorSpeed;
        measuredActuatorTorque = torqueCurrent;

        previousEncoderPositionTick = position;
    }

    public double getMeasuredEncoderPosition()
    {
        return measuredEncoderPosition;
    }

    public double getMeasuredPosition()
    {
        return measuredActuatorPosition;
    }

    public double getMeasuredVelocity()
    {
        return measuredActuatorVelocity;
    }

    public double getMeasuredTorque()
    {
        return measuredActuatorTorque;
    }

    public int getMeasuredTemperature()
    {
        return measuredTemperature;
    }

    public static int getID(TPCANMsg message)
    {
        return message.getData()[0];
    }

}
