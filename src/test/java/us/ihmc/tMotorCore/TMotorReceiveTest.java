package us.ihmc.tMotorCore;

import org.junit.jupiter.api.Test;
import peak.can.basic.TPCANMsg;
import us.ihmc.can.CANTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.etherCAT.javalution.Struct;
import us.ihmc.tMotorCore.CANMessages.TMotorReply;
import us.ihmc.tMotorCore.parameters.TMotorAK109Parameters;
import us.ihmc.tMotorCore.parameters.TMotorAK606Parameters;
import us.ihmc.tMotorCore.parameters.TMotorAK809Parameters;
import us.ihmc.tMotorCore.parameters.TMotorParameters;

import java.nio.ByteBuffer;

import static org.junit.jupiter.api.Assertions.*;

public class TMotorReceiveTest
{
   public static void main(String[] args)
   {
      Class<?>[] applicationClasses = new Class[] {TMotorReply.class, Status.class};
      Class<?>[] testClasses = new Class[] {TMotorReceiveTest.class};
      MutationTestFacilitator.facilitateMutationTestForClasses(applicationClasses, testClasses);
   }

   @Test
   public void compareArrays()
   {
      compareBuffers(new TMotorAK109Parameters());
      compareBuffers(new TMotorAK606Parameters());
      compareBuffers(new TMotorAK809Parameters());

      testOverMax(new TMotorAK109Parameters());
      testOverMax(new TMotorAK606Parameters());
      testOverMax(new TMotorAK809Parameters());
   }

   private void compareBuffers(TMotorParameters motorParameters)
   {
      TMotorReply reply = new TMotorReply(motorParameters);
      Status javalutionStatus = new Status();

      double maxPosition = motorParameters.getPositionLimitUpper();
      double minPosition = motorParameters.getPositionLimitLower();

      double velocityLimitUpper = motorParameters.getVelocityLimitUpper();
      double velocityLimitLower = motorParameters.getVelocityLimitLower();

      double torqueLimitUpper = motorParameters.getTorqueLimitUpper();
      double torqueLimitLower = motorParameters.getTorqueLimitLower();

      boolean success = true;
      for (int i = 0; i < 10000; i++)
      {
         int id = (int) (Math.random() * 255);

         double position = Math.random() * maxPosition * 2.0 - maxPosition;
         int rawPosition = CANTools.double_to_uint(position, minPosition, maxPosition, TMotorReply.BITS_POSITION);

         double velocity = Math.random() * velocityLimitUpper * 2.0 - velocityLimitUpper;
         int rawVelocity = CANTools.double_to_uint(velocity, velocityLimitLower, velocityLimitUpper, TMotorReply.BITS_VELOCITY);

         double torque = Math.random() * torqueLimitUpper * 2.0 - torqueLimitUpper;
         int rawTorque = CANTools.double_to_uint(torque, torqueLimitLower, torqueLimitUpper, TMotorReply.BITS_TORQUE);

         int temp = (int) (Math.random() * 255);
         int error = (int) (Math.random() * 255);

         ByteBuffer byteBuffer = javalutionStatus.getByteBuffer();
         byteBuffer.clear();

         javalutionStatus.id.set((byte) id);
         javalutionStatus.position.set(rawPosition);
         javalutionStatus.velocity.set((short) rawVelocity);
         javalutionStatus.current.set((short) rawTorque);
         javalutionStatus.temp.set((short) temp);
         javalutionStatus.error.set((short) error);

         byte[] javalutionBuffer = new byte[8];
         byteBuffer.get(javalutionBuffer);

         TPCANMsg canMsg = new TPCANMsg();
         canMsg.setData(javalutionBuffer, (byte) 8);
         reply.parseAndUnpack(canMsg);

         double posPrecision = (maxPosition - minPosition) / Math.pow(2.0, TMotorReply.BITS_POSITION) * 1.001;
         if (Math.abs(reply.getMeasuredPosition() - position) > posPrecision)
         {
            success = false;
            System.out.println("Pos: " + reply.getMeasuredPosition() + "," + position);
         }

         double velPrecision = (velocityLimitUpper - velocityLimitLower) / Math.pow(2.0, TMotorReply.BITS_VELOCITY) * 1.001;
         if (Math.abs(reply.getMeasuredVelocity() - velocity) > velPrecision)
         {
            success = false;
            System.out.println("Vel: " + reply.getMeasuredVelocity() + "," + velocity);
         }

         if (rawTorque != reply.getMeasuredTorqueRaw())
         {
            success = false;
            System.out.println("CURRENT: " + rawTorque + "," + reply.getMeasuredTorqueRaw());
         }

         double torquePrecision = (torqueLimitUpper - torqueLimitLower) / Math.pow(2.0, TMotorReply.BITS_TORQUE) * 1.001;
         if (Math.abs(reply.getMeasuredTorque() - torque) > torquePrecision)
         {
            success = false;
            System.out.println("CURRENT: " + reply.getMeasuredTorque() + "," + torque);
         }
      }
      assertTrue(success);
   }

   private void testOverMax(TMotorParameters motorParameters)
   {
      TMotorReply reply = new TMotorReply(motorParameters);
      Status javalutionStatus = new Status();

      double maxPosition = motorParameters.getPositionLimitUpper();
      double minPosition = motorParameters.getPositionLimitLower();

      double velocityLimitUpper = motorParameters.getVelocityLimitUpper();
      double velocityLimitLower = motorParameters.getVelocityLimitLower();

      double torqueLimitUpper = motorParameters.getTorqueLimitUpper();
      double torqueLimitLower = motorParameters.getTorqueLimitLower();

      boolean success = true;
      for (int i = 0; i < 10000; i++)
      {
         int id = (int) (Math.random() * 255);

         double position = Math.random() * maxPosition * 4.0 - maxPosition * 2.0;
         int rawPosition = CANTools.double_to_uint(position, minPosition, maxPosition, TMotorReply.BITS_POSITION);

         double velocity = Math.random() * velocityLimitUpper * 4.0 - velocityLimitUpper * 2.0;
         int rawVelocity = CANTools.double_to_uint(velocity, velocityLimitLower, velocityLimitUpper, TMotorReply.BITS_VELOCITY);

         double torque = Math.random() * torqueLimitUpper * 4.0 - torqueLimitUpper * 2.0;
         int rawTorque = CANTools.double_to_uint(torque, torqueLimitLower, torqueLimitUpper, TMotorReply.BITS_TORQUE);

         int temp = (int) (Math.random() * 255);
         int error = (int) (Math.random() * 255);

         ByteBuffer byteBuffer = javalutionStatus.getByteBuffer();
         byteBuffer.clear();

         javalutionStatus.id.set((byte) id);
         javalutionStatus.position.set(rawPosition);
         javalutionStatus.velocity.set((short) rawVelocity);
         javalutionStatus.current.set((short) rawTorque);
         javalutionStatus.temp.set((short) temp);
         javalutionStatus.error.set((short) error);

         byte[] javalutionBuffer = new byte[8];
         byteBuffer.get(javalutionBuffer);

         TPCANMsg canMsg = new TPCANMsg();
         canMsg.setData(javalutionBuffer, (byte) 8);
         reply.parseAndUnpack(canMsg);

         double posPrecision = (maxPosition - minPosition) / Math.pow(2.0, TMotorReply.BITS_POSITION) * 1.001;
         double clampedPosition = MathTools.clamp(position, minPosition, maxPosition);

         if (Math.abs(reply.getMeasuredPosition() - clampedPosition) > posPrecision)
         {
            success = false;
            System.out.println("Pos: " + reply.getMeasuredPosition() + "," + clampedPosition);
         }

         double velPrecision = (velocityLimitUpper - velocityLimitLower) / Math.pow(2.0, TMotorReply.BITS_VELOCITY) * 1.001;
         double clampedVelocity = MathTools.clamp(velocity, velocityLimitLower, velocityLimitUpper);

         if (Math.abs(reply.getMeasuredVelocity() - clampedVelocity) > velPrecision)
         {
            success = false;
            System.out.println("Vel: " + reply.getMeasuredVelocity() + "," + clampedVelocity);
         }

         if (rawTorque != reply.getMeasuredTorqueRaw())
         {
            success = false;
            System.out.println("CURRENT: " + rawTorque + "," + reply.getMeasuredTorqueRaw());
         }

         double clampedTorque = MathTools.clamp(torque, torqueLimitLower, torqueLimitUpper);
         double torquePrecision = (torqueLimitUpper - torqueLimitLower) / Math.pow(2.0, TMotorReply.BITS_TORQUE) * 1.001;
         if (Math.abs(reply.getMeasuredTorque() - clampedTorque) > torquePrecision)
         {
            success = false;
            System.out.println("CURRENT: " + reply.getMeasuredTorque() + "," + clampedTorque);
         }
      }
      assertTrue(success);
   }

   public class Status extends Struct
   {
      public final Unsigned8 id = new Unsigned8();
      public final Unsigned16 position = new Unsigned16();
      public final BitField velocity = new BitField(12);
      public final BitField current = new BitField(12);
      public final Unsigned8 temp = new Unsigned8();
      public final Unsigned8 error = new Unsigned8();

      @Override
      public boolean isPacked()
      {
         return true;
      }
   }
}
