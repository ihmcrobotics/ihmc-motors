package us.ihmc.tMotorCore;

import org.junit.jupiter.api.Test;
import us.ihmc.can.CANTools;
import us.ihmc.etherCAT.javalution.Struct;
import us.ihmc.tMotorCore.CANMessages.TMotorCommand;
import us.ihmc.tMotorCore.parameters.TMotorAK109Parameters;
import us.ihmc.tMotorCore.parameters.TMotorAK606Parameters;
import us.ihmc.tMotorCore.parameters.TMotorAK809Parameters;
import us.ihmc.tMotorCore.parameters.TMotorParameters;

import java.nio.ByteBuffer;

import static org.junit.jupiter.api.Assertions.*;

public class TMotorCommandTest
{
   @Test
   public void compareArrays()
   {
      compareBuffers(new TMotorAK109Parameters());
      compareBuffers(new TMotorAK606Parameters());
      compareBuffers(new TMotorAK809Parameters());
   }

   @Test
   public void testPackingAndUnpackingWriteBuffers()
   {
      testPackAndUnpack(new TMotorAK109Parameters());
      testPackAndUnpack(new TMotorAK606Parameters());
      testPackAndUnpack(new TMotorAK809Parameters());

      testPack(new TMotorAK109Parameters());
      testPack(new TMotorAK606Parameters());
      testPack(new TMotorAK809Parameters());
   }

   private void compareBuffers(TMotorParameters motorParameters)
   {
      TMotorCommand cmd = new TMotorCommand(6, motorParameters);

      Command javalutionCMD = new Command();

      double maxPosition = motorParameters.getPositionLimitUpper();
      double minPosition = motorParameters.getPositionLimitLower();

      double velocityLimitUpper = motorParameters.getVelocityLimitUpper();
      double velocityLimitLower = motorParameters.getVelocityLimitLower();

      double maximumKp = motorParameters.getMaximumKp();
      double maximumKd = motorParameters.getMaximumKd();

      double torqueLimitUpper = motorParameters.getTorqueLimitUpper();
      double torqueLimitLower = motorParameters.getTorqueLimitLower();

      for (int i = 0; i < 10000; i++)
      {
         double desiredPosition = Math.random() * maxPosition * 2.0 - maxPosition;
         int rawDesiredPosition = CANTools.double_to_uint(desiredPosition, minPosition, maxPosition, TMotorCommand.BITS_POSITION);

         double desiredVelocity = Math.random() * velocityLimitUpper * 2.0 - velocityLimitUpper;
         int rawDesiredVelocity = CANTools.double_to_uint(desiredVelocity, velocityLimitLower, velocityLimitUpper, TMotorCommand.BITS_VELOCITY);

         double desiredKp = Math.random() * maximumKp;
         int rawDesiredKp = CANTools.double_to_uint(desiredKp, 0.0, maximumKp, TMotorCommand.BITS_KP);

         double desiredKd = Math.random() * maximumKd;
         int rawDesiredKd = CANTools.double_to_uint(desiredKd, 0.0, maximumKd, TMotorCommand.BITS_KD);

         double desiredTorque = Math.random() * torqueLimitUpper * 2.0 - torqueLimitUpper;
         int rawDesiredTorque = CANTools.double_to_uint(desiredTorque, torqueLimitLower, torqueLimitUpper, TMotorCommand.BITS_TORQUE);

         ByteBuffer byteBuffer = javalutionCMD.getByteBuffer();
         byteBuffer.clear();

         javalutionCMD.position.set(rawDesiredPosition);
         javalutionCMD.velocity.set((short) rawDesiredVelocity);
         javalutionCMD.kp.set((short) rawDesiredKp);
         javalutionCMD.kd.set((short) rawDesiredKd);
         javalutionCMD.current.set((short) rawDesiredTorque);

         cmd.setCommand(desiredPosition, desiredVelocity, desiredTorque, desiredKp, desiredKd);
         byte[] userCommand = cmd.getUserCommand();

         byte[] javalutionBuffer = new byte[8];
         byteBuffer.get(javalutionBuffer);

         for (int index = 0; index < userCommand.length; index++)
         {
            assertEquals(userCommand[index], javalutionBuffer[index]);
         }
      }
   }

   private void testPackAndUnpack(TMotorParameters motorParameters)
   {
      TMotorCommand cmd = new TMotorCommand(6, motorParameters);

      Command javalutionCMD = new Command();

      double maxPosition = motorParameters.getPositionLimitUpper();
      double minPosition = motorParameters.getPositionLimitLower();

      double velocityLimitUpper = motorParameters.getVelocityLimitUpper();
      double velocityLimitLower = motorParameters.getVelocityLimitLower();

      double maximumKp = motorParameters.getMaximumKp();
      double maximumKd = motorParameters.getMaximumKd();

      double torqueLimitUpper = motorParameters.getTorqueLimitUpper();
      double torqueLimitLower = motorParameters.getTorqueLimitLower();

      boolean success = true;

      for (int i = 0; i < 10000; i++)
      {
         double desiredPosition = Math.random() * maxPosition * 2.0 - maxPosition;
         int rawDesiredPosition = CANTools.double_to_uint(desiredPosition, minPosition, maxPosition, TMotorCommand.BITS_POSITION);

         double desiredVelocity = Math.random() * velocityLimitUpper * 2.0 - velocityLimitUpper;
         int rawDesiredVelocity = CANTools.double_to_uint(desiredVelocity, velocityLimitLower, velocityLimitUpper, TMotorCommand.BITS_VELOCITY);

         double desiredKp = Math.random() * maximumKp;
         int rawDesiredKp = CANTools.double_to_uint(desiredKp, 0.0, maximumKp, TMotorCommand.BITS_KP);

         double desiredKd = Math.random() * maximumKd;
         int rawDesiredKd = CANTools.double_to_uint(desiredKd, 0.0, maximumKd, TMotorCommand.BITS_KD);

         double desiredTorque = Math.random() * torqueLimitUpper * 2.0 - torqueLimitUpper;
         int rawDesiredTorque = CANTools.double_to_uint(desiredTorque, torqueLimitLower, torqueLimitUpper, TMotorCommand.BITS_TORQUE);

         javalutionCMD.position.set(rawDesiredPosition);
         javalutionCMD.velocity.set((short) rawDesiredVelocity);
         javalutionCMD.kp.set((short) rawDesiredKp);
         javalutionCMD.kd.set((short) rawDesiredKd);
         javalutionCMD.current.set((short) rawDesiredTorque);

         //pos
         int rawDesiredPositionFromCommand = javalutionCMD.position.get();
         double desiredPositionFromCommand = CANTools.uint_to_double(rawDesiredPositionFromCommand, minPosition, maxPosition, TMotorCommand.BITS_POSITION);

         if (rawDesiredPosition != rawDesiredPositionFromCommand)
         {
            success = false;
            System.out.println("Pos: " + rawDesiredPosition + "," + rawDesiredPositionFromCommand);
         }

         double posPrecision = (maxPosition - minPosition) / Math.pow(2.0, TMotorCommand.BITS_POSITION) * 1.001;
         if (Math.abs(desiredPosition - desiredPositionFromCommand) > posPrecision)
         {
            success = false;
            System.out.println("Pos: " + desiredPosition + "," + desiredPositionFromCommand);
         }

         //vel
         short rawDesiredVelocityFromCommand = javalutionCMD.velocity.shortValue();
         double desiredVelocityFromCommand = CANTools.uint_to_double(rawDesiredVelocityFromCommand,
                                                                     velocityLimitLower,
                                                                     velocityLimitUpper,
                                                                     TMotorCommand.BITS_VELOCITY);

         if (rawDesiredVelocity != rawDesiredVelocityFromCommand)
         {
            success = false;
            System.out.println("Vel: " + rawDesiredVelocity + "," + rawDesiredVelocityFromCommand);
         }

         double velPrecision = (velocityLimitUpper - velocityLimitLower) / Math.pow(2.0, TMotorCommand.BITS_VELOCITY) * 1.001;
         if (Math.abs(desiredVelocity - desiredVelocityFromCommand) > velPrecision)
         {
            success = false;
            System.out.println("Vel: " + desiredVelocity + "," + desiredVelocityFromCommand);
         }

         //kp
         short rawDesiredKpFromCommand = javalutionCMD.kp.shortValue();
         double desiredKpFromCommand = CANTools.uint_to_double(rawDesiredKpFromCommand, 0.0, maximumKp, TMotorCommand.BITS_KP);

         if (rawDesiredKp != rawDesiredKpFromCommand)
         {
            success = false;
            System.out.println("KP: " + rawDesiredKp + "," + rawDesiredKpFromCommand);
         }

         double kpPrecision = maximumKp / Math.pow(2.0, TMotorCommand.BITS_KP) * 1.001;
         if (Math.abs(desiredKp - desiredKpFromCommand) > kpPrecision)
         {
            success = false;
            System.out.println("KP: " + desiredKp + "," + desiredKpFromCommand);
         }

         //kd
         short rawDesiredKdFromCommand = javalutionCMD.kd.shortValue();
         double desiredKdFromCommand = CANTools.uint_to_double(rawDesiredKdFromCommand, 0.0, maximumKd, TMotorCommand.BITS_KD);

         if (rawDesiredKd != rawDesiredKdFromCommand)
         {
            success = false;
            System.out.println("KD: " + rawDesiredKd + "," + rawDesiredKdFromCommand);
         }

         double kdPrecision = maximumKd / Math.pow(2.0, TMotorCommand.BITS_KD) * 1.001;
         if (Math.abs(desiredKd - desiredKdFromCommand) > kdPrecision)
         {
            success = false;
            System.out.println("KD: " + desiredKd + "," + desiredKdFromCommand);
         }

         //kd
         short rawDesiredCurrentFromCommand = javalutionCMD.current.shortValue();
         double desiredCurrentFromCommand = CANTools.uint_to_double(rawDesiredCurrentFromCommand,
                                                                    torqueLimitLower,
                                                                    torqueLimitUpper,
                                                                    TMotorCommand.BITS_TORQUE);

         if (rawDesiredTorque != rawDesiredCurrentFromCommand)
         {
            success = false;
            System.out.println("CURRENT: " + rawDesiredTorque + "," + rawDesiredCurrentFromCommand);
         }

         double torquePrecision = (torqueLimitUpper - torqueLimitLower) / Math.pow(2.0, TMotorCommand.BITS_TORQUE) * 1.001;
         if (Math.abs(desiredTorque - desiredCurrentFromCommand) > torquePrecision)
         {
            success = false;
            System.out.println("CURRENT: " + desiredTorque + "," + desiredCurrentFromCommand);
         }
      }

      assertTrue(success, "Something was wrong with packing and unpacking data from the Javalution byte buffers");
   }

   private void testPack(TMotorParameters motorParameters)
   {
      TMotorCommand cmd = new TMotorCommand(6, motorParameters);

      double maxPosition = motorParameters.getPositionLimitUpper();
      double minPosition = motorParameters.getPositionLimitLower();

      double velocityLimitUpper = motorParameters.getVelocityLimitUpper();
      double velocityLimitLower = motorParameters.getVelocityLimitLower();

      double maximumKp = motorParameters.getMaximumKp();
      double maximumKd = motorParameters.getMaximumKd();

      double torqueLimitUpper = motorParameters.getTorqueLimitUpper();
      double torqueLimitLower = motorParameters.getTorqueLimitLower();

      boolean success = true;

      for (int i = 0; i < 10000; i++)
      {
         double desiredPosition = Math.random() * maxPosition * 2.0 - maxPosition;
         int rawDesiredPosition = CANTools.double_to_uint(desiredPosition, minPosition, maxPosition, TMotorCommand.BITS_POSITION);

         double desiredVelocity = Math.random() * velocityLimitUpper * 2.0 - velocityLimitUpper;
         int rawDesiredVelocity = CANTools.double_to_uint(desiredVelocity, velocityLimitLower, velocityLimitUpper, TMotorCommand.BITS_VELOCITY);

         double desiredKp = Math.random() * maximumKp;
         int rawDesiredKp = CANTools.double_to_uint(desiredKp, 0.0, maximumKp, TMotorCommand.BITS_KP);

         double desiredKd = Math.random() * maximumKd;
         int rawDesiredKd = CANTools.double_to_uint(desiredKd, 0.0, maximumKd, TMotorCommand.BITS_KD);

         double desiredTorque = Math.random() * torqueLimitUpper * 2.0 - torqueLimitUpper;
         int rawDesiredTorque = CANTools.double_to_uint(desiredTorque, torqueLimitLower, torqueLimitUpper, TMotorCommand.BITS_TORQUE);

         cmd.setCommand(desiredPosition, desiredVelocity, desiredTorque, desiredKp, desiredKd);

         byte[] userCommand = cmd.getUserCommand();

         int data0 = Byte.toUnsignedInt(userCommand[0]);
         int data1 = Byte.toUnsignedInt(userCommand[1]);
         int data2 = Byte.toUnsignedInt(userCommand[2]);
         int data3 = Byte.toUnsignedInt(userCommand[3]);
         int data4 = Byte.toUnsignedInt(userCommand[4]);
         int data5 = Byte.toUnsignedInt(userCommand[5]);
         int data6 = Byte.toUnsignedInt(userCommand[6]);
         int data7 = Byte.toUnsignedInt(userCommand[7]);

         int rawDesiredPositionFromCommand = (data0 << 8) | data1;
         int rawDesiredVelocityFromCommand = (data2 << 4) | (data3 >> 4);
         int rawDesiredKpFromCommand = ((data3 & 0xF) << 8) | data4;
         int rawDesiredKdFromCommand = (data5 << 4) | (data6 >> 4);
         int rawDesiredCurrentFromCommand = ((data6 & 0xF) << 8) | data7;

         //        Convert data from raw counts to proper units
         double desiredPositionFromCommand = CANTools.uint_to_double(rawDesiredPositionFromCommand,
                                                                     motorParameters.getPositionLimitLower(),
                                                                     motorParameters.getPositionLimitUpper(),
                                                                     TMotorCommand.BITS_POSITION);

         double desiredVelocityFromCommand = CANTools.uint_to_double(rawDesiredVelocityFromCommand,
                                                                     motorParameters.getVelocityLimitLower(),
                                                                     motorParameters.getVelocityLimitUpper(),
                                                                     TMotorCommand.BITS_VELOCITY);

         double desiredKpFromCommand = CANTools.uint_to_double(rawDesiredKpFromCommand, 0.0, maximumKp, TMotorCommand.BITS_KP);

         double desiredKdFromCommand = CANTools.uint_to_double(rawDesiredKdFromCommand, 0.0, maximumKd, TMotorCommand.BITS_KD);

         double desiredCurrentFromCommand = CANTools.uint_to_double(rawDesiredCurrentFromCommand,
                                                                    motorParameters.getTorqueLimitLower(),
                                                                    motorParameters.getTorqueLimitUpper(),
                                                                    TMotorCommand.BITS_TORQUE);

         //pos
         if (rawDesiredPosition != rawDesiredPositionFromCommand)
         {
            success = false;
            System.out.println("Pos: " + rawDesiredPosition + "," + rawDesiredPositionFromCommand);
         }

         double posPrecision = (maxPosition - minPosition) / Math.pow(2.0, TMotorCommand.BITS_POSITION) * 1.001;
         if (Math.abs(desiredPosition - desiredPositionFromCommand) > posPrecision)
         {
            success = false;
            System.out.println("Pos: " + desiredPosition + "," + desiredPositionFromCommand);
         }

         //vel
         if (rawDesiredVelocity != rawDesiredVelocityFromCommand)
         {
            success = false;
            System.out.println("Vel: " + rawDesiredVelocity + "," + rawDesiredVelocityFromCommand);
         }

         double velPrecision = (velocityLimitUpper - velocityLimitLower) / Math.pow(2.0, TMotorCommand.BITS_VELOCITY) * 1.001;
         if (Math.abs(desiredVelocity - desiredVelocityFromCommand) > velPrecision)
         {
            success = false;
            System.out.println("Vel: " + desiredVelocity + "," + desiredVelocityFromCommand);
         }

         //kp
         if (rawDesiredKp != rawDesiredKpFromCommand)
         {
            success = false;
            System.out.println("KP: " + rawDesiredKp + "," + rawDesiredKpFromCommand);
         }

         double kpPrecision = maximumKp / Math.pow(2.0, TMotorCommand.BITS_KP) * 1.001;
         if (Math.abs(desiredKp - desiredKpFromCommand) > kpPrecision)
         {
            success = false;
            System.out.println("KP: " + desiredKp + "," + desiredKpFromCommand);
         }

         //kd

         if (rawDesiredKd != rawDesiredKdFromCommand)
         {
            success = false;
            System.out.println("KD: " + rawDesiredKd + "," + rawDesiredKdFromCommand);
         }

         double kdPrecision = maximumKd / Math.pow(2.0, TMotorCommand.BITS_KD) * 1.001;
         if (Math.abs(desiredKd - desiredKdFromCommand) > kdPrecision)
         {
            success = false;
            System.out.println("KD: " + desiredKd + "," + desiredKdFromCommand);
         }

         //torque
         if (rawDesiredTorque != rawDesiredCurrentFromCommand)
         {
            success = false;
            System.out.println("CURRENT: " + rawDesiredTorque + "," + rawDesiredCurrentFromCommand);
         }

         double torquePrecision = (torqueLimitUpper - torqueLimitLower) / Math.pow(2.0, TMotorCommand.BITS_TORQUE) * 1.001;
         if (Math.abs(desiredTorque - desiredCurrentFromCommand) > torquePrecision)
         {
            success = false;
            System.out.println("CURRENT: " + desiredTorque + "," + desiredCurrentFromCommand);
         }
      }

      assertTrue(success, "Something was wrong with packing and unpacking data from the Javalution byte buffers");
   }

   public class Command extends Struct
   {
      public final Unsigned16 position = new Unsigned16();
      public final BitField velocity = new BitField(12);
      public final BitField kp = new BitField(12);
      public final BitField kd = new BitField(12);
      public final BitField current = new BitField(12);

      @Override
      public boolean isPacked()
      {
         return true;
      }
   }
}
