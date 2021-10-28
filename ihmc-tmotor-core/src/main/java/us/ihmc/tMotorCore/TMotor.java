package us.ihmc.tMotorCore;

import peak.can.basic.TPCANMsg;
import us.ihmc.CAN.CANMotor;
import us.ihmc.tMotorCore.CANMessages.TMotorCANReceiveMessage;
import us.ihmc.tMotorCore.CANMessages.TMotorCANReplyMessage;
import us.ihmc.tMotorCore.parameters.TMotorParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public class TMotor extends CANMotor
{
   private final TMotorCANReceiveMessage motorReceiveMsg;      // CAN message sent to motor
   private final TMotorCANReplyMessage motorReplyMsg;          // CAN message reply from motor
   private TPCANMsg commandedMsg;

   /**
    * The T-Motor firmware uses at Kt=0.095 Nm/A, but this is under-approximated. This multiplier can be used to adjust to the actual Kt.
    *
    * Desired torques will be scaled by this value before being sent to the motor controller, and the motor controller's reported torque
    * will be scaled down by this value.
    */
   private double torqueScale = 1.0;

   public TMotor(int ID, String name, TMotorVersion version, double dt, YoRegistry parentRegistry)
   {
      super(ID, name, dt);

      TMotorParameters encoderParameters = version.getMotorParameters();
      motorReceiveMsg =  new TMotorCANReceiveMessage(ID, encoderParameters);
      motorReplyMsg = new TMotorCANReplyMessage(encoderParameters);
      velocityFilterCoefficient.set(0.9);

      motorDirection.set(1);
      parentRegistry.addChild(registry);
   }

   @Override
   public void read(TPCANMsg message)
   {
      yoCANMsg.setReceived(message);
      motorReplyMsg.parseAndUnpack(message);

      measuredEncoderPosition.set(motorReplyMsg.getMeasuredEncoderPosition());
      measuredActuatorPosition.set(motorDirection.getValue() * motorReplyMsg.getMeasuredPosition());
      measuredVelocity.set(motorDirection.getValue() * motorReplyMsg.getMeasuredVelocity());
      measuredTorque.set(motorDirection.getValue() * motorReplyMsg.getMeasuredTorque() / torqueScale);
      filteredVelocity.update();
      filteredTorque.update();
   }

   public void update()
   {

   }

   public void parseAndPack(int kp, int kd, float desiredPosition, float desiredVelocity, float desiredTorque)
   {
      motorReceiveMsg.parseAndPackControlMsg((float) motorDirection.getValue() * desiredPosition,
                                             (float) motorDirection.getValue() * desiredVelocity,
                                             (float) (motorDirection.getValue() * desiredTorque * torqueScale),
                                             kp,
                                             kd);
   }

   public void setCommandedMsg(TPCANMsg receiveMsg)
   {
      commandedMsg = receiveMsg;
   }

   public void reversePositiveMotorDirection() { motorDirection.set(-1); }

   public TPCANMsg getDisableMotorMsg()
   {
      return motorReceiveMsg.getDisableMotorMsg();
   }

   public TPCANMsg getEnableMotorMsg()
   {
      return motorReceiveMsg.getEnableMotorMsg();
   }

   public TPCANMsg getZeroMotorMsg()
   {
      return motorReceiveMsg.getZeroMotorMsg();
   }

   public TPCANMsg getControlMotorMsg()
   {
      return motorReceiveMsg.getControlMotorMsg();
   }

   public TPCANMsg getCommandedMsg()
   {
      return this.commandedMsg;
   }

   public double getPosition()
   {
      return measuredActuatorPosition.getDoubleValue();
   }

   public double getVelocity()
   {
      return measuredVelocity.getDoubleValue();
   }

   public double getFilteredVelocity()
   {
      return filteredVelocity.getValue();
   }

   public double getTorque()
   {
      return measuredTorque.getDoubleValue();
   }

   public double getFilteredTorque()
   {
      return filteredTorque.getDoubleValue();
   }

   public int getDirection()
   {
      return motorDirection.getIntegerValue();
   }

   public void setTorqueScale(double torqueScaling)
   {
      this.torqueScale = torqueScaling;
   }
}