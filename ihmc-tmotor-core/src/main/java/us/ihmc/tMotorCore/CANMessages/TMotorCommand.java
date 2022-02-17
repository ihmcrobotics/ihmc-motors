package us.ihmc.tMotorCore.CANMessages;

import peak.can.basic.TPCANMessageType;
import peak.can.basic.TPCANMsg;
import us.ihmc.CAN.CANTools;
import us.ihmc.commons.MathTools;
import us.ihmc.tMotorCore.parameters.TMotorParameters;

/**
 * This represents the CAN Command message to send to the TMotor when in MIT Control mode See
 * https://store.cubemars.com/images/file/20211201/1638329381542610.pdf for more details
 */
public class TMotorCommand
{
   private static final byte STANDARD_CAN_MESSAGE = TPCANMessageType.PCAN_MESSAGE_STANDARD.getValue();
   private static final int BITS_POSITION = 16;
   private static final int BITS_VELOCITY = 12;
   private static final int BITS_TORQUE = 12;
   private static final int BITS_KP = 12;
   private static final int BITS_KD = 12;

   private final byte[] enterMotorCommand = new byte[] {(byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFC};
   private final byte[] exitMotorCommand = new byte[] {(byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFD};
   private final byte[] zeroMotorCommand = new byte[] {(byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFE};
   private final byte[] userCommand = new byte[] {(byte) 0x7F, (byte) 0xFF, (byte) 0x7F, (byte) 0xF0, (byte) 0x00, (byte) 0x00, (byte) 0x07, (byte) 0xFF};

   private final TPCANMsg pcanMsg;
   private final TMotorParameters motorParameters;
   
   private int desiredPositionRaw;
   private int desiredVelocityRaw;
   private int desiredTorqueRaw;
   private int kpRaw;
   private int kdRaw;

   public TMotorCommand(int ID, TMotorParameters motorParameters)
   {
      this.motorParameters = motorParameters;
      pcanMsg = new TPCANMsg(ID, STANDARD_CAN_MESSAGE, (byte) 8, userCommand);
   }

   public void setCommand(double desiredPosition, double desiredVelocity, double desiredTorque, double kp, double kd)
   {
      desiredPosition = MathTools.clamp(desiredPosition, motorParameters.getPositionLimitLower(), motorParameters.getPositionLimitUpper());
      desiredVelocity = MathTools.clamp(desiredVelocity, motorParameters.getVelocityLimitLower(), motorParameters.getVelocityLimitUpper());
      desiredTorque = MathTools.clamp(desiredTorque, motorParameters.getTorqueLimitLower(), motorParameters.getTorqueLimitUpper());

      desiredPositionRaw = CANTools.double_to_uint(desiredPosition,
                                                       motorParameters.getPositionLimitLower(),
                                                       motorParameters.getPositionLimitUpper(),
                                                       BITS_POSITION);

      desiredVelocityRaw = CANTools.double_to_uint(desiredVelocity,
                                                       motorParameters.getVelocityLimitLower(),
                                                       motorParameters.getVelocityLimitUpper(),
                                                       BITS_VELOCITY);

      desiredTorqueRaw = CANTools.double_to_uint(desiredTorque, motorParameters.getTorqueLimitLower(), motorParameters.getTorqueLimitUpper(), BITS_TORQUE);
      kpRaw = CANTools.double_to_uint(kp, 0, motorParameters.getMaximumKp(), BITS_KP);
      kdRaw = CANTools.double_to_uint(kd, 0, motorParameters.getMaximumKd(), BITS_KD);

      userCommand[0] = (byte) (desiredPositionRaw >> 8);
      userCommand[1] = (byte) (desiredPositionRaw & 0xFF);
      userCommand[2] = (byte) (desiredVelocityRaw >> 4);
      userCommand[3] = (byte) (((desiredVelocityRaw & 0xF) << 4) | (kpRaw >> 8));
      userCommand[4] = (byte) (kpRaw & 0xFF);
      userCommand[5] = (byte) (kdRaw >> 4);
      userCommand[6] = (byte) (((kdRaw & 0xF) << 4) | desiredTorqueRaw >> 8);
      userCommand[7] = (byte) (desiredTorqueRaw & 0xFF);

      pcanMsg.setData(userCommand, (byte) 8);
   }

   public void setCommandToEnableMotor()
   {
      pcanMsg.setData(enterMotorCommand, (byte) 8);
   }

   public void setCommandToDisableMotor()
   {
      pcanMsg.setData(exitMotorCommand, (byte) 8);
   }

   public void setCommandToZeroMotor()
   {
      pcanMsg.setData(zeroMotorCommand, (byte) 8);
   }

   public TPCANMsg getCANMsg()
   {
      return pcanMsg;
   }

   public int getDesiredPositionRaw()
   {
      return desiredPositionRaw;
   }
   
   public int getDesiredVelocityRaw()
   {
      return desiredVelocityRaw;
   }
   
   public int getDesiredTorqueRaw()
   {
      return desiredTorqueRaw;
   }
   
   public int getKpRaw()
   {
      return kpRaw;
   }
   
   public int getKdRaw()
   {
      return kdRaw;
   }
}
