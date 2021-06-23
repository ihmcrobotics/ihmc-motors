package us.ihmc.tMotorCore;

import peak.can.basic.TPCANMsg;
import us.ihmc.CAN.CANMotor;
import us.ihmc.tMotorCore.CANMessages.TMotorCANReceiveMessage;
import us.ihmc.tMotorCore.CANMessages.TMotorCANReplyMessage;
import us.ihmc.tMotorCore.parameters.TMotorParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class TMotor extends CANMotor
{
   private final TMotorLowLevelController controller;
   private static final double UNSAFE_SPEED = 6.0;

   // Command messages for T-Motor
   private final TMotorCANReceiveMessage motorReceiveMsg;      // CAN message sent to motor
   private final TMotorCANReplyMessage motorReplyMsg;          // CAN message reply from motor

   private final YoBoolean sendEnableMotorCommand;
   private final YoBoolean sendDisableMotorCommand;
   private final YoBoolean sendZeroMotorCommand;

   public TMotor(int ID, TMotorVersion version, double dt, YoDouble time, YoRegistry parentRegistry)
   {
      super(ID, dt, time, parentRegistry);
      String prefix = ID + "_";

      TMotorParameters encoderParameters = version.getMotorParameters();
      motorReceiveMsg =  new TMotorCANReceiveMessage(ID, encoderParameters);
      motorReplyMsg = new TMotorCANReplyMessage(encoderParameters);

      controller = new TMotorLowLevelController(prefix, parentRegistry);

      velocityFilterCoefficient.setVariableBounds(0.0, 1.0);
      velocityFilterCoefficient.set(0.2);
      desiredActuatorPosition.set(0.0);

      sendEnableMotorCommand = new YoBoolean(prefix + "sendEnableMotorCommand", registry);
      sendDisableMotorCommand = new YoBoolean(prefix + "sendDisableMotorCommand", registry);
      sendZeroMotorCommand = new YoBoolean(prefix + "sendZeroMotorCommand", registry);

      parentRegistry.addChild(registry);
   }

   public TPCANMsg requestRead()
   {
      return motorReceiveMsg.getEnableMotorMsg();
   }

   @Override
   public void read(TPCANMsg message)
   {
      yoCANMsg.setReceived(message);

      motorReplyMsg.parseAndUnpack(message);

      measuredActuatorPosition.set(motorReplyMsg.getMeasuredPosition());
      measuredVelocity.set(motorReplyMsg.getMeasuredVelocity());
      measuredTorqueCurrent.set(motorReplyMsg.getMeasuredTorque());

      filteredVelocity.update();
   }

   @Override
   public void update()
   {
      controller.setDesireds(functionGenerator.getValue(), functionGenerator.getValueDot());
      controller.doControl();
      updateYoDesireds();
   }

   private void updateYoDesireds()
   {
      desiredActuatorPosition.set(controller.getDesiredPosition());
      desiredActuatorVelocity.set(controller.getDesiredVelocity());
      desiredActuatorTorque.set(controller.getDesiredTorque());
   }

   public void setMeasuredForce(double measuredForce)
   {
      controller.setMeasuredForce(measuredForce);
   }

   @Override
   public TPCANMsg write()
   {
      if(motorIsInUnsafeState())
      {
         yoCANMsg.setSent(motorReceiveMsg.getDisableMotorCommandData());
         return motorReceiveMsg.getDisableMotorMsg();
      }

      if (sendEnableMotorCommand.getBooleanValue())
      {
         yoCANMsg.setSent(motorReceiveMsg.getEnableMotorCommandData());
         sendEnableMotorCommand.set(false);
         return motorReceiveMsg.getEnableMotorMsg();
      }

      if (sendDisableMotorCommand.getBooleanValue())
      {
         yoCANMsg.setSent(motorReceiveMsg.getDisableMotorCommandData());
         sendDisableMotorCommand.set(false);
         return motorReceiveMsg.getDisableMotorMsg();
      }

      if (sendZeroMotorCommand.getBooleanValue())
      {
         yoCANMsg.setSent(motorReceiveMsg.getZeroMotorCommandData());
         sendZeroMotorCommand.set(false);
         return motorReceiveMsg.getZeroMotorMsg();
      }

      int kp = controller.getKp();
      int kd = controller.getKd();
      float desiredPosition = (float) desiredActuatorPosition.getDoubleValue();
      float desiredVelocity = (float) desiredActuatorVelocity.getDoubleValue();
      float desiredTorque = (float) desiredActuatorTorque.getDoubleValue();
      motorReceiveMsg.parseAndPackControlMsg(desiredPosition, desiredVelocity, desiredTorque, kp, kd);
      yoCANMsg.setSent(motorReceiveMsg.getControlMotorCommandData());

      return motorReceiveMsg.getControlMotorMsg();
   }

   private boolean motorIsInUnsafeState()
   {
      if( Math.abs(measuredVelocity.getDoubleValue()) > UNSAFE_SPEED)
         return true;

      return false;
   }

   public int getID()
   {
      return ID;
   }

   public double getDesiredTorque()
   {
      return desiredActuatorTorque.getDoubleValue();
   }

   public double getPosition()
   {
      return measuredActuatorPosition.getDoubleValue();
   }

   public double getVelocity()
   {
      return measuredVelocity.getDoubleValue();
   }

   public double getTorque()
   {
      return measuredTorqueCurrent.getDoubleValue();
   }
}
