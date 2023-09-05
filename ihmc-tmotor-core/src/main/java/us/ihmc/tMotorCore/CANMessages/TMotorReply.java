package us.ihmc.tMotorCore.CANMessages;

import peak.can.basic.TPCANMsg;
import us.ihmc.CAN.CANTools;
import us.ihmc.tMotorCore.parameters.TMotorParameters;

/**
 * This represents the CAN status message the TMotor sends back when in MIT Control mode See
 * https://store.cubemars.com/images/file/20211201/1638329381542610.pdf for more details
 */
public class TMotorReply
{
   public static final int BITS_POSITION = 16;
   public static final int BITS_VELOCITY = 12;
   public static final int BITS_TORQUE = 12;

   private TMotorParameters motorParameters;

   private int id;
   
   private int measuredPositionRaw;
   private int measuredVelocityRaw;
   private int measuredTorqueRaw;
   
   private double measuredPosition;
   private double measuredVelocity;
   private double measuredTorque;

   /**
    * Primary constructor. TMotorParameters is used to scale the CAN MSG data. If you find scaling
    * issues, please confirm the max values match the settings on the motor
    * 
    * @param motorParameters
    */
   public TMotorReply(TMotorParameters motorParameters)
   {
      this.motorParameters = motorParameters;
   }

   public void parseAndUnpack(TPCANMsg message)
   {
      int data1 = Byte.toUnsignedInt(message.getData()[1]);
      int data2 = Byte.toUnsignedInt(message.getData()[2]);
      int data3 = Byte.toUnsignedInt(message.getData()[3]);
      int data4 = Byte.toUnsignedInt(message.getData()[4]);
      int data5 = Byte.toUnsignedInt(message.getData()[5]);

      // parse message
      id = message.getData()[0];
      measuredPositionRaw = (data1 << 8) | data2;
      measuredVelocityRaw = (data3 << 4) | (data4 >> 4);
      measuredTorqueRaw = ((data4 & 0xF) << 8) | data5;

      // Convert data from raw counts to proper units
      measuredPosition = CANTools.uint_to_double(measuredPositionRaw,
                                                 motorParameters.getPositionLimitLower(),
                                                 motorParameters.getPositionLimitUpper(),
                                                 BITS_POSITION);

      measuredVelocity = CANTools.uint_to_double(measuredVelocityRaw,
                                                 motorParameters.getVelocityLimitLower(),
                                                 motorParameters.getVelocityLimitUpper(),
                                                 BITS_VELOCITY);

      measuredTorque = CANTools.uint_to_double(measuredTorqueRaw,
                                               motorParameters.getTorqueLimitLower(),
                                               motorParameters.getTorqueLimitUpper(),
                                               BITS_TORQUE);
   }

   public int getId()
   {
      return id;
   }

   public double getMeasuredPosition()
   {
      return measuredPosition;
   }

   public double getMeasuredVelocity()
   {
      return measuredVelocity;
   }

   public double getMeasuredTorque()
   {
      return measuredTorque;
   }
   
   public int getMeasuredPositionRaw()
   {
      return measuredPositionRaw;
   }

   public int getMeasuredVelocityRaw()
   {
      return measuredVelocityRaw;
   }

   public int getMeasuredTorqueRaw()
   {
      return measuredTorqueRaw;
   }
}
