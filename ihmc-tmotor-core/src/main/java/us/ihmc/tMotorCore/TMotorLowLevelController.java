package us.ihmc.tMotorCore;

import peak.can.basic.TPCANMsg;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.teststands.TestBedControlMode;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class TMotorLowLevelController implements RobotController
{
   private final YoRegistry registry;

   private double unsafeOutputSpeed;

   private final YoDouble desiredMotorPosition;
   private final YoDouble desiredMotorVelocity;
   private final YoDouble desiredMotorTorque;
   //private final YoDouble desiredMotorTorqueHalved;

   private final TMotor tMotor;
   private double pulleyRadius = 0;

   private final YoDouble measuredPosition;
   private final YoDouble measuredForce;

   private final YoBoolean sendEnableMotorCommand;
   private final YoBoolean sendDisableMotorCommand;
   private final YoBoolean sendZeroMotorCommand;

   // internal motor PID controller gains
   private final YoInteger motorPositionKp;
   private final YoInteger motorVelocityKd;
   private final YoDouble motorTorqueKp;

   private final YoDouble positionError;
   private final YoDouble torqueError;

   //external PID Controller
   private final YoEnum<MotorControlMode> motorControlMode;
   private final PIDController externalPositionPIDController;
   private final PIDController externalVelocityPIDController;
   private final PIDController externalTorquePIDController;
   /*private final YoDouble controllerPositionKp;
   private final YoDouble controllerPositionKi;
   private final YoDouble controllerPositionKd;
   private final YoDouble controllerVelocityKp;
   private final YoDouble controllerVelocityKi;
   private final YoDouble controllerVelocityKd;
   private final YoDouble controllerTorqueKp;
   private final YoDouble controllerTorqueKi;
   private final YoDouble controllerTorqueKd;*/


   public TMotorLowLevelController(String name, TMotor tMotor, YoRegistry parentRegistry)
   {
      this.tMotor = tMotor;
      this.registry = new YoRegistry(getClass().getSimpleName() + "_" + name);

      desiredMotorPosition = new YoDouble(name + "_desiredMotorPosition", registry);
      desiredMotorVelocity = new YoDouble(name + "_desiredMotorVelocity", registry);
      desiredMotorTorque = new YoDouble(name + "_desiredMotorTorque", registry);
      //desiredMotorTorqueHalved = new YoDouble(name + "_desiredMotorTorqueHalved",registry);

      sendEnableMotorCommand = new YoBoolean(name + "_sendEnableMotorCommand", registry);
      sendDisableMotorCommand = new YoBoolean(name + "_sendDisableMotorCommand", registry);
      sendZeroMotorCommand = new YoBoolean(name + "_sendZeroMotorCommand", registry);

      measuredPosition = new YoDouble(name + "_measuredPosition", registry);
      measuredForce = new YoDouble(name + "_measuredForce", registry);

      motorPositionKp = new YoInteger(name + "_motorPositionKp", registry);
      motorVelocityKd = new YoInteger(name + "_motorVelocityKd", registry);
      motorTorqueKp = new YoDouble(name + "_motorTorqueKp", registry);


      positionError = new YoDouble(name + "_positionError", registry);
      torqueError = new YoDouble(name + "_torqueError", registry);

      motorControlMode = new YoEnum<>(name + "motorControlModeSelect", registry, MotorControlMode.class);
      controllerPositionKp = new YoDouble(name + "_controllerPositionKp", registry);
      controllerPositionKi = new YoDouble(name + "_controllerPositionKi", registry);
      controllerPositionKd = new YoDouble(name + "_controllerPositionKd", registry);
      controllerVelocityKp = new YoDouble(name + "_controllerVelocityKp", registry);
      controllerVelocityKi = new YoDouble(name + "_controllerVelocityKi", registry);
      controllerVelocityKd = new YoDouble(name + "_controllerVelocityKd", registry);
      controllerTorqueKp = new YoDouble(name + "_controllerTorqueKp", registry);
      controllerTorqueKi = new YoDouble(name + "_controllerTorqueKi", registry);
      controllerTorqueKd = new YoDouble(name + "_controllerTorqueKd", registry);
      externalPositionPIDController = new PIDController()

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

      float desiredPosition = (float) desiredMotorPosition.getDoubleValue();
      float desiredVelocity = (float) desiredMotorVelocity.getDoubleValue();
      float desiredTorque = (float) desiredMotorTorque.getDoubleValue();

      // TODO: why is this +=? and why is desired position changing with position error?
//      desiredPosition += controllerPositionKp.getDoubleValue() * getPositionError();
//      desiredTorque += controllerTorqueKp.getDoubleValue() * getTorqueError();

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
      positionError.set(desiredMotorPosition.getDoubleValue() - measuredPosition.getDoubleValue());
      return positionError.getDoubleValue();
   }

   public double getTorqueError()
   {
      torqueError.set(desiredMotorTorque.getDoubleValue() - (measuredForce.getDoubleValue() * pulleyRadius));
      return torqueError.getDoubleValue();
   }

   public void setUnsafeOutputSpeed(double unsafeSpeed)
   {
      unsafeOutputSpeed = unsafeSpeed;
   }

   public void setTorqueScale(double torqueScale)
   {
      tMotor.setTorqueScale(torqueScale);
   }

   public void setDesiredPosition(double position)
   {
      desiredMotorPosition.set(position);
   }

   public void setDesiredVelocity(double velocity)
   {
      desiredMotorVelocity.set(velocity);
   }

   public void setDesiredTorque(double torque)
   {
      desiredMotorTorque.set(torque);
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

   public void sendEnableMotorCommand()
   {
      sendEnableMotorCommand.set(true);
   }

   public void sendDisableMotorCommand()
   {
      sendDisableMotorCommand.set(true);
   }
}
