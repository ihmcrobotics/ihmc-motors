package us.ihmc.tMotorCore.CANMessages;

import peak.can.basic.TPCANMessageType;
import peak.can.basic.TPCANMsg;
import us.ihmc.CAN.CANTools;
import us.ihmc.tMotorCore.parameters.TMotorParameters;

import java.util.HashMap;

public class TMotorCANReceiveMessage
{
    private static final byte STANDARD_CAN_MESSAGE = TPCANMessageType.PCAN_MESSAGE_STANDARD.getValue();

    private enum MotorStatus{ENABLE_MOTOR, DISABLE_MOTOR, ZERO_MOTOR_POSITION, CONTROL_MOTOR}
    private final HashMap<MotorStatus, TPCANMsg> motorCANControlMsg = new HashMap<>();

    // commands that are updated at each tick
    private byte[] updatedMotorControlCommand = new byte[] {0x7F, (byte) 0xFF, 0x7F, (byte) 0xF0, 0x00, 0x00, 0x07, (byte) 0xFF};
    private TPCANMsg controlMotorCommandMsg;

    private final float MIN_POSITION;
    private final float MAX_POSITION;
    private final float MIN_VELOCITY;
    private final float MAX_VELOCITY;
    private final float MIN_TORQUE;
    private final float MAX_TORQUE;
    private final float MAXIMUM_KP;
    private final float MAXIMUM_KD;

    public TMotorCANReceiveMessage(int ID, TMotorParameters encoderParameters)
    {
        initializeMotorCommands(ID);
        MIN_POSITION = encoderParameters.getMinimumEncoderPosition();
        MAX_POSITION = encoderParameters.getMaximumEncoderPosition();
        MIN_VELOCITY = encoderParameters.getMinimumEncoderVelocity();
        MAX_VELOCITY = encoderParameters.getMaximumEncoderVelocity();
        MIN_TORQUE = encoderParameters.getMinimumTorqueReading();
        MAX_TORQUE = encoderParameters.getMaximumTorqueReading();
        MAXIMUM_KP = encoderParameters.getMaximumKp();
        MAXIMUM_KD = encoderParameters.getMaximumKd();
    }

    private void initializeMotorCommands(int ID)
    {
        byte[] enterMotorCommand = new byte[] {(byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFC};
        byte[] exitMotorCommand = new byte[] {(byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFD};
        byte[] zeroMotorCommand = new byte[] {(byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFE};

        TPCANMsg enterControlModeMsg = new TPCANMsg(ID, STANDARD_CAN_MESSAGE, (byte) 8, enterMotorCommand);
        TPCANMsg exitControlModeMsg = new TPCANMsg(ID, STANDARD_CAN_MESSAGE, (byte) 8, exitMotorCommand);
        TPCANMsg zeroMotorPositionMsg = new TPCANMsg(ID, STANDARD_CAN_MESSAGE, (byte) 8, zeroMotorCommand);
        controlMotorCommandMsg = new TPCANMsg(ID, STANDARD_CAN_MESSAGE, (byte) 8, updatedMotorControlCommand);

        motorCANControlMsg.put(MotorStatus.ENABLE_MOTOR, enterControlModeMsg);
        motorCANControlMsg.put(MotorStatus.DISABLE_MOTOR, exitControlModeMsg);
        motorCANControlMsg.put(MotorStatus.ZERO_MOTOR_POSITION, zeroMotorPositionMsg);
        motorCANControlMsg.put(MotorStatus.CONTROL_MOTOR, controlMotorCommandMsg);
    }

    public void parseAndPackControlMsg(float desiredPosition, float desiredVelocity, float desiredTorque, float kp, float kd)
    {
        desiredPosition = Math.min(Math.max(MIN_POSITION, desiredPosition), MAX_POSITION);
        desiredVelocity = Math.min(Math.max(MIN_VELOCITY, desiredVelocity), MAX_VELOCITY);
        desiredTorque = Math.min(Math.max(MIN_TORQUE, desiredTorque), MAX_TORQUE);
        int desiredPositionInt = CANTools.float_to_uint(desiredPosition, MIN_POSITION, MAX_POSITION, 16);
        int desiredVelocityInt = CANTools.float_to_uint(desiredVelocity, MIN_VELOCITY, MAX_VELOCITY, 12);
        int desiredTorqueInt = CANTools.float_to_uint(desiredTorque, MIN_TORQUE, MAX_TORQUE, 12);
        int kpInt = CANTools.float_to_uint(kp, 0, MAXIMUM_KP, 12);
        int kdInt = CANTools.float_to_uint(kd, 0, MAXIMUM_KD, 12);

        updatedMotorControlCommand[0] = (byte) (desiredPositionInt >> 8);
        updatedMotorControlCommand[1] = (byte) (desiredPositionInt & 0xFF);
        updatedMotorControlCommand[2] = (byte) (desiredVelocityInt >> 4);
        updatedMotorControlCommand[3] = (byte) (((desiredVelocityInt & 0xF) << 4) | (kpInt >> 8));
        updatedMotorControlCommand[4] = (byte) (kpInt & 0xFF);
        updatedMotorControlCommand[5] = (byte) (kdInt >> 4);
        updatedMotorControlCommand[6] = (byte) (((kdInt & 0xF) << 4) | desiredTorqueInt >> 8);
        updatedMotorControlCommand[7] = (byte) (desiredTorqueInt & 0xFF);

        controlMotorCommandMsg.setData(updatedMotorControlCommand, (byte) 8);
        motorCANControlMsg.replace(MotorStatus.CONTROL_MOTOR, controlMotorCommandMsg);
    }

    public TPCANMsg getEnableMotorMsg()
    {
        return motorCANControlMsg.get(MotorStatus.ENABLE_MOTOR);
    }

    public TPCANMsg getDisableMotorMsg()
    {
        return motorCANControlMsg.get(MotorStatus.DISABLE_MOTOR);
    }

    public TPCANMsg getControlMotorMsg()
    {
        return motorCANControlMsg.get(MotorStatus.CONTROL_MOTOR);
    }

    public TPCANMsg getZeroMotorMsg()
    {
        return motorCANControlMsg.get(MotorStatus.ZERO_MOTOR_POSITION);
    }

    public byte[] getEnableMotorCommandData()
    {
        return controlMotorCommandMsg.getData();
    }

    public byte[] getDisableMotorCommandData()
    {
        return controlMotorCommandMsg.getData();
    }

    public byte[] getControlMotorCommandData()
    {
        return controlMotorCommandMsg.getData();
    }

    public byte[] getZeroMotorCommandData()
    {
        return controlMotorCommandMsg.getData();
    }
}
