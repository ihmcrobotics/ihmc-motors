package us.ihmc.tMotorCore;

import peak.can.basic.TPCANMsg;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class TMotorLowLevelController
{
   private final YoRegistry registry;

   private final YoDouble unsafeOutputSpeed;

   private final TMotor tMotor;
   private final YoBoolean sendEnableMotorCommand;
   private final YoBoolean sendDisableMotorCommand;
   private final YoBoolean sendZeroMotorCommand;
   
   private final YoLong resendEnableCounter;
   private final YoLong resendDisableCounter;
   private final YoLong numberOfTimesToResendCommands;

   // Desireds
   private final YoDouble desiredActuatorPosition;
   private final YoDouble desiredActuatorVelocity;
   private final YoDouble desiredActuatorTorque;

   // Gains
   private final YoDouble motorPositionKp;
   private final YoDouble motorVelocityKd;

   public TMotorLowLevelController(String name, TMotor tMotor, YoRegistry parentRegistry)
   {
      this.tMotor = tMotor;
      this.registry = new YoRegistry(getClass().getSimpleName() + "_" + name);

      desiredActuatorPosition = new YoDouble(name + "_desiredActuatorPosition", registry);
      desiredActuatorVelocity = new YoDouble(name + "_desiredActuatorVelocity", registry);
      desiredActuatorTorque = new YoDouble(name + "_desiredActuatorTorque", registry);
      unsafeOutputSpeed = new YoDouble(name + "_unsafeOutputSpeed", registry);

      sendEnableMotorCommand = new YoBoolean(name + "_sendEnableMotorCommand", registry);
      sendDisableMotorCommand = new YoBoolean(name + "_sendDisableMotorCommand", registry);
      sendZeroMotorCommand = new YoBoolean(name + "_sendZeroMotorCommand", registry);
      
      resendEnableCounter = new YoLong("resendEnableCounter", registry);
      resendDisableCounter = new YoLong("resendDisableCounter", registry);
      numberOfTimesToResendCommands = new YoLong("numberOfTimesToResendCommands", registry);
      numberOfTimesToResendCommands.set(2);

      motorPositionKp = new YoDouble(name + "_motorPositionKp", registry);
      motorVelocityKd = new YoDouble(name + "_motorVelocityKd", registry);

      parentRegistry.addChild(registry);
   }

   public void read(TPCANMsg receivedMsg)
   {
      tMotor.read(receivedMsg);
   }

   public void doControl()
   {
      if (sendDisableMotorCommand.getBooleanValue())
      {
         tMotor.setCommandToDisableMotor();
         resendDisableCounter.increment();
         
         if(resendDisableCounter.getLongValue() >= numberOfTimesToResendCommands.getLongValue())
         {
            sendDisableMotorCommand.set(false);
            resendDisableCounter.set(0);
         }
         return;
      }
      
      if (sendEnableMotorCommand.getBooleanValue())
      {
         tMotor.setCommandToEnableMotor();
         resendEnableCounter.increment();
         
         if(resendEnableCounter.getLongValue() >= numberOfTimesToResendCommands.getLongValue())
         {
            sendEnableMotorCommand.set(false);
            resendEnableCounter.set(0);
         }
         
         return;
      }

      if (sendZeroMotorCommand.getBooleanValue())
      {
         tMotor.setCommandToZeroEncoder();
         sendZeroMotorCommand.set(false);
         return;
      }

      double desiredPosition = desiredActuatorPosition.getDoubleValue();
      double desiredVelocity = desiredActuatorVelocity.getDoubleValue();
      double desiredTorque = desiredActuatorTorque.getDoubleValue();
      double kp = motorPositionKp.getDoubleValue();
      double kd = motorVelocityKd.getDoubleValue();

      tMotor.setCommand(kp, kd, desiredPosition, desiredVelocity, desiredTorque);
   }

   public void setUnsafeOutputSpeed(double unsafeSpeed)
   {
      unsafeOutputSpeed.set(unsafeSpeed);
   }

   public void setTorqueScale(double torqueScale)
   {
      tMotor.setTorqueScale(torqueScale);
   }

   public void setDesiredPosition(double position)
   {
      desiredActuatorPosition.set(position);
   }

   public void setDesiredVelocity(double velocity)
   {
      desiredActuatorVelocity.set(velocity);
   }

   public void setDesiredTorque(double torque)
   {
      desiredActuatorTorque.set(torque);
   }

   public void setKp(double kp)
   {
      motorPositionKp.set(kp);
   }

   public void setKd(double kd)
   {
      motorVelocityKd.set(kd);
   }

   public double getMeasuredPosition()
   {
      return tMotor.getPosition();
   }

   public double getMeasuredVelocity()
   {
      return tMotor.getVelocity();
   }

   public double getMeasuredVelocityFiltered()
   {
      return tMotor.getFilteredVelocity();
   }

   public double getMeasuredTorque()
   {
      return tMotor.getTorque();
   }

   public void sendEnableMotorCommand()
   {
      sendEnableMotorCommand.set(true);
   }

   public void sendDisableMotorCommand()
   {
      sendDisableMotorCommand.set(true);
   }

   public TMotor getTMotor()
   {
      return tMotor;
   }
}
