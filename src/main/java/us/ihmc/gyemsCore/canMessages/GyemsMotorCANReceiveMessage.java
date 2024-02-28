package us.ihmc.gyemsCore.canMessages;

import peak.can.basic.TPCANMessageType;
import peak.can.basic.TPCANMsg;
import us.ihmc.can.CANTools;
import us.ihmc.gyemsCore.parameters.GyemsMotorParameters;

import java.util.HashMap;

public class GyemsMotorCANReceiveMessage
{
   private static final byte STANDARD_CAN_MESSAGE = TPCANMessageType.PCAN_MESSAGE_STANDARD.getValue();
   // static messages
   private static final byte[] requestMotorCommands2 = new byte[] {(byte) 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
   private static final byte[] requestMotorPosition = new byte[] {(byte) 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
   private final HashMap<MotorCommands, TPCANMsg> motorCANControlMsg = new HashMap<>();
   private final int GEAR_RATIO_TO_ONE;
   private final int ENCODER_POSITION_RESOLUTION;
   // commands that are updated at each tick
   private byte[] speedLimitedPositionControlCommand = new byte[] {(byte) 0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
   private byte[] requestWritePID = new byte[] {(byte) 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
   private TPCANMsg controlMotorPositionMsg;
   private TPCANMsg writePIDToRAMMsg;
   private double maximumActuatorSpeed;
   public GyemsMotorCANReceiveMessage(int ID, GyemsMotorParameters encoderParameters)
   {
      initializeMotorCommands(ID);
      GEAR_RATIO_TO_ONE = encoderParameters.getGearRatioToOne();
      ENCODER_POSITION_RESOLUTION = encoderParameters.getEncoderPositionResolution();
      maximumActuatorSpeed = Math.toDegrees(18.0); // in deg/sec
   }

   private void initializeMotorCommands(int ID)
   {
      byte[] requestWriteMotorOffsetPosition = new byte[] {(byte) 0x91, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

      TPCANMsg readMotorCommands2Msg = new TPCANMsg(ID, STANDARD_CAN_MESSAGE, (byte) 8, requestMotorCommands2);
      TPCANMsg readMotorPositionMsg = new TPCANMsg(ID, STANDARD_CAN_MESSAGE, (byte) 8, requestMotorPosition);
      TPCANMsg writeMotorOffsetMsg = new TPCANMsg(ID, STANDARD_CAN_MESSAGE, (byte) 8, requestWriteMotorOffsetPosition);
      writePIDToRAMMsg = new TPCANMsg(ID, STANDARD_CAN_MESSAGE, (byte) 8, requestWritePID);
      controlMotorPositionMsg = new TPCANMsg(ID, STANDARD_CAN_MESSAGE, (byte) 8, speedLimitedPositionControlCommand);

      motorCANControlMsg.put(MotorCommands.READ_MOTOR_STATUS, readMotorCommands2Msg);
      motorCANControlMsg.put(MotorCommands.READ_MOTOR_POSITION, readMotorPositionMsg);
      motorCANControlMsg.put(MotorCommands.WRITE_PID_GAINS, writePIDToRAMMsg);
      motorCANControlMsg.put(MotorCommands.WRITE_MOTOR_OFFSET, writeMotorOffsetMsg);
      motorCANControlMsg.put(MotorCommands.POSITION_CONTROL_SPEED_LIMITED, controlMotorPositionMsg);
   }

   public void parseAndPackControlMsg(double desiredPosition, double desiredVelocity, double desiredTorque)
   {
      int maxSpeed = (int) (GEAR_RATIO_TO_ONE * maximumActuatorSpeed);
      int desiredMotorPosition = CANTools.actuatorToMotorPosition(desiredPosition, GEAR_RATIO_TO_ONE, ENCODER_POSITION_RESOLUTION);

      byte speedLimitLow = (byte) (maxSpeed & 0xFF);
      byte speedLimitHigh = (byte) ((maxSpeed >> 8) & 0xFF);
      //        int speedLimitHighInt = speedLimitHigh;
      //        maximumMotorSpeed.set( (speedLimitHighInt<<8) | speedLimitLow );

      byte positionLow = (byte) (desiredMotorPosition & 0xFF);
      byte positionMidLow = (byte) (desiredMotorPosition >> 8 & 0xFF);
      byte positionMidHigh = (byte) (desiredMotorPosition >> 16 & 0xFF);
      byte positionHigh = (byte) (desiredMotorPosition >> 24 & 0xFF);

      speedLimitedPositionControlCommand[2] = speedLimitLow;
      speedLimitedPositionControlCommand[3] = speedLimitHigh;
      speedLimitedPositionControlCommand[4] = positionLow;
      speedLimitedPositionControlCommand[5] = positionMidLow;
      speedLimitedPositionControlCommand[6] = positionMidHigh;
      speedLimitedPositionControlCommand[7] = positionHigh;

      controlMotorPositionMsg.setData(speedLimitedPositionControlCommand, (byte) 8);
      motorCANControlMsg.replace(MotorCommands.POSITION_CONTROL_SPEED_LIMITED, controlMotorPositionMsg);
   }

   public boolean updatePIDGains(byte posKp, byte posKi, byte velKp, byte velKi, byte tauKp, byte tauKi)
   {
      requestWritePID[2] = posKp;
      requestWritePID[3] = posKi;
      requestWritePID[4] = velKp;
      requestWritePID[5] = velKi;
      requestWritePID[6] = tauKp;
      requestWritePID[7] = tauKi;
      writePIDToRAMMsg.setData(requestWritePID, (byte) 8);
      motorCANControlMsg.replace(MotorCommands.WRITE_PID_GAINS, writePIDToRAMMsg);
      return true;
   }

   public TPCANMsg getControlMotorPositionMsg()
   {
      return motorCANControlMsg.get(MotorCommands.POSITION_CONTROL_SPEED_LIMITED);
   }

   public TPCANMsg getUpdatePIDGainsMsg()
   {
      return motorCANControlMsg.get(MotorCommands.WRITE_PID_GAINS);
   }

   public TPCANMsg getRequestReadMsg()
   {
      return motorCANControlMsg.get(MotorCommands.READ_MOTOR_STATUS);
   }

   public byte[] getControlMotorPositionCommandData()
   {
      return controlMotorPositionMsg.getData();
   }

   private enum MotorCommands
   {
      READ_MOTOR_STATUS, READ_MOTOR_POSITION, WRITE_PID_GAINS, WRITE_MOTOR_OFFSET, POSITION_CONTROL_SPEED_LIMITED
   }
}
