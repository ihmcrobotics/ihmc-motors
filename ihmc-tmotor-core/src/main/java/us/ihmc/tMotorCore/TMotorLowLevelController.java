package us.ihmc.tMotorCore;

import peak.can.basic.TPCANMsg;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class TMotorLowLevelController implements RobotController
{
   private final YoRegistry registry;

   private double unsafeOutputSpeed;

   private final YoDouble desiredActuatorPosition;
   private final YoDouble desiredActuatorVelocity;
   private final YoDouble desiredActuatorTorque;

   private final TMotor tMotor;
   private double pulleyRadius = 0;

   private final YoDouble controllerPositionKp;
   private final YoDouble controllerTorqueKp;
   private final YoDouble measuredPosition;
   private final YoDouble measuredForce;

   private final YoBoolean sendEnableMotorCommand;
   private final YoBoolean sendDisableMotorCommand;
   private final YoBoolean sendZeroMotorCommand;

   // gains
   private final YoInteger motorPositionKp;
   private final YoInteger motorVelocityKd;
   private final YoDouble motorTorqueKp;

   private final YoDouble positionError;
   private final YoDouble torqueError;

   public TMotorLowLevelController(String name, TMotor tMotor, YoRegistry parentRegistry)
   {
      this.tMotor = tMotor;
      this.registry = new YoRegistry(getClass().getSimpleName() + "_" + name);

      desiredActuatorPosition = new YoDouble(name + "_desiredActuatorPosition", registry);
      desiredActuatorVelocity = new YoDouble(name + "_desiredActuatorVelocity", registry);
      desiredActuatorTorque = new YoDouble(name + "_desiredActuatorTorque", registry);

      sendEnableMotorCommand = new YoBoolean(name + "_sendEnableMotorCommand", registry);
      sendDisableMotorCommand = new YoBoolean(name + "_sendDisableMotorCommand", registry);
      sendZeroMotorCommand = new YoBoolean(name + "_sendZeroMotorCommand", registry);

      measuredPosition = new YoDouble(name + "_measuredPosition", registry);
      measuredForce = new YoDouble(name + "_measuredForce", registry);

      motorPositionKp = new YoInteger(name + "_motorPositionKp", registry);
      motorVelocityKd = new YoInteger(name + "_motorVelocityKd", registry);
      motorTorqueKp = new YoDouble(name + "_motorTorqueKp", registry);
      controllerPositionKp = new YoDouble(name + "_controllerPositionKp", registry);
      controllerTorqueKp = new YoDouble(name + "_controllerTorqueKp", registry);

      positionError = new YoDouble(name + "_positionError", registry);
      torqueError = new YoDouble(name + "_torqueError", registry);

      parentRegistry.addChild(registry);
   }

   /**
    * Computes command to send to T-Motor
    */
   @Override
   public void doControl()
   {
      if (isUserSendingPredefinedCommand() || isMotorInUnsafeState())
         return;

      float desiredPosition = (float) desiredActuatorPosition.getDoubleValue();
      float desiredVelocity = (float) desiredActuatorVelocity.getDoubleValue();
      float desiredTorque = (float) desiredActuatorTorque.getDoubleValue();

      // TODO: why is this +=? and why is desired position changing with position error?
      desiredPosition += controllerPositionKp.getDoubleValue() * getPositionError();
      desiredTorque += controllerTorqueKp.getDoubleValue() * getTorqueError();

      tMotor.parseAndPack(getKp(), getKd(), desiredPosition, desiredVelocity, desiredTorque);
      tMotor.getYoCANMsg().setSent(tMotor.getControlMotorMsg().getData());
      tMotor.setCommandedMsg(tMotor.getControlMotorMsg());
   }

   private boolean isUserSendingPredefinedCommand()
   {
      if (sendEnableMotorCommand.getBooleanValue())
      {
         tMotor.setCommandedMsg(tMotor.getEnableMotorMsg());
         tMotor.getYoCANMsg().setSent(tMotor.getEnableMotorMsg().getData());
         sendEnableMotorCommand.set(false);
         return true;
      }

      if (sendDisableMotorCommand.getBooleanValue())
      {
         tMotor.setCommandedMsg(tMotor.getDisableMotorMsg());
         tMotor.getYoCANMsg().setSent(tMotor.getDisableMotorMsg().getData());
         sendDisableMotorCommand.set(false);
         return true;
      }

      if (sendZeroMotorCommand.getBooleanValue())
      {
         tMotor.setCommandedMsg(tMotor.getZeroMotorMsg());
         tMotor.getYoCANMsg().setSent(tMotor.getZeroMotorMsg().getData());
         sendZeroMotorCommand.set(false);
         return true;
      }
      return false;
   }

   private boolean isMotorInUnsafeState()
   {
      if (motorIsInUnsafeState())
      {
         tMotor.setCommandedMsg(tMotor.getDisableMotorMsg());
         tMotor.getYoCANMsg().setSent(tMotor.getDisableMotorMsg().getData());
         return true;
      }
      return false;
   }

   private boolean motorIsInUnsafeState()
   {
      if (Math.abs(tMotor.getVelocity()) > unsafeOutputSpeed)
         return true;

      return false;
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public void read(TPCANMsg receivedMsg)
   {
      tMotor.read(receivedMsg);
   }

   public void update()
   {
      tMotor.update();
   }

   public void updateMeasuredForce(double measuredForce)
   {
      this.measuredForce.set(measuredForce);
   }

   public void updateMeasuredPosition(double measuredPosition)
   {
      this.measuredPosition.set(measuredPosition);
   }

   public double getPositionError()
   {
      positionError.set(desiredActuatorPosition.getDoubleValue() - measuredPosition.getDoubleValue());
      return positionError.getDoubleValue();
   }

   public double getTorqueError()
   {
      torqueError.set(desiredActuatorTorque.getDoubleValue() - (measuredForce.getDoubleValue() * pulleyRadius));
      return torqueError.getDoubleValue();
   }

   public void setUnsafeOutputSpeed(double unsafeSpeed)
   {
      unsafeOutputSpeed = unsafeSpeed;
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
      motorPositionKp.set((int) kp);
   }

   public void setKd(double kd)
   {
      motorVelocityKd.set((int) kd);
   }

   public TMotor getMotor()
   {
      return this.tMotor;
   }

   public void setPulleyRadius(double pulleyRadius)
   {
      this.pulleyRadius = pulleyRadius;
   }

   public double getMeasuredAppliedTorque()
   {
      return tMotor.getTorque();
   }

   public double getMotorPosition()
   {
      return tMotor.getPosition();
   }

   public double getMotorVelocity()
   {
      return tMotor.getVelocity();
   }

   public double getMotorTorque()
   {
      return tMotor.getTorque();
   }

   public int getKp()
   {
      return motorPositionKp.getIntegerValue();
   }

   public int getKd()
   {
      return motorVelocityKd.getIntegerValue();
   }

   public double getTorqueKp()
   {
      return this.motorTorqueKp.getDoubleValue();
   }
}
