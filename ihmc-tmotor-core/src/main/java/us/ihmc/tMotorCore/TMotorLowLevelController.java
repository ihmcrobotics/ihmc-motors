package us.ihmc.tMotorCore;

import peak.can.basic.TPCANMsg;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class TMotorLowLevelController implements RobotController
{
   private final YoRegistry registry;

   private double unsafeOutputSpeed;
   private double DT;

   private final YoDouble desiredMotorPosition;
   private final YoDouble desiredMotorVelocity;
   private final YoDouble desiredMotorTorque;
   private final YoDouble desiredMotorTorqueRate;
   private final YoDouble commandedMotorTorque;
   private final YoDouble commandedMotorSpeed;

   //private final YoDouble desiredMotorTorqueHalved;

   private final TMotor tMotor;
   private double pulleyRadius = 0;

   private final YoDouble measuredPosition;
   private final YoDouble measuredVelocity;
   private final YoDouble measuredTorque;

   private final YoBoolean sendEnableMotorCommand;
   private final YoBoolean sendDisableMotorCommand;
   private final YoBoolean sendZeroMotorCommand;

   // internal motor PID controller gains
   private final YoInteger motorPositionKp;
   private final YoInteger motorVelocityKd;

   private final YoDouble positionError;
   private final YoDouble velocityError;
   private final YoDouble torqueError;
   private double lastMeasuredTorque;

   //external PID Controller
   private final YoEnum<MotorControlMode> motorControlMode;
   private final PIDController externalPositionPIDController;
   private final PIDController externalTorquePIDController;
   private final YoPIDGains externalPositionGains;
   private final YoPIDGains externalTorqueGains;

   /*private final YoDouble controllerPositionKp;
   private final YoDouble controllerPositionKi;
   private final YoDouble controllerPositionKd;
   private final YoDouble controllerVelocityKp;
   private final YoDouble controllerVelocityKi;
   private final YoDouble controllerVelocityKd;
   private final YoDouble controllerTorqueKp;
   private final YoDouble controllerTorqueKi;
   private final YoDouble controllerTorqueKd;*/

   public TMotorLowLevelController(String name, TMotor tMotor, YoRegistry parentRegistry){
      this(name, tMotor, parentRegistry,  0.001);
   }

   public TMotorLowLevelController(String name, TMotor tMotor, YoRegistry parentRegistry, double DT)
   {
      this.tMotor = tMotor;
      this.registry = new YoRegistry(getClass().getSimpleName() + "_" + name);

      desiredMotorPosition = new YoDouble(name + "_desiredMotorPosition", registry);
      desiredMotorVelocity = new YoDouble(name + "_desiredMotorVelocity", registry);
      desiredMotorTorque = new YoDouble(name + "_desiredMotorTorque", registry);
      desiredMotorTorqueRate = new YoDouble(name + "_desiredMotorTorqueRate", registry);
      commandedMotorTorque = new YoDouble(name + "_commandedMotorTorque", registry);
      commandedMotorSpeed = new YoDouble(name + "_commandedMotorSpeed", registry);

      sendEnableMotorCommand = new YoBoolean(name + "_sendEnableMotorCommand", registry);
      sendDisableMotorCommand = new YoBoolean(name + "_sendDisableMotorCommand", registry);
      sendZeroMotorCommand = new YoBoolean(name + "_sendZeroMotorCommand", registry);

      measuredPosition = new YoDouble(name + "_measuredPosition", registry);
      measuredVelocity = new YoDouble(name + "_measuredVelocity", registry);
      measuredTorque = new YoDouble(name + "_measuredTorque", registry);

      motorPositionKp = new YoInteger(name + "_motorPositionKp", registry);
      motorVelocityKd = new YoInteger(name + "_motorVelocityKd", registry);


      positionError = new YoDouble(name + "_positionError", registry);
      velocityError = new YoDouble(name + "_velocityError", registry);
      torqueError = new YoDouble(name + "_torqueError", registry);

      motorControlMode = new YoEnum<>(name + "motorControlModeSelect", registry, MotorControlMode.class);
      motorControlMode.set(MotorControlMode.INTERNAL_MOTOR_CONTROLLER);
      externalPositionGains = new YoPIDGains("_"+ name + "_externalPositionController", registry);
      externalTorqueGains = new YoPIDGains("_"+ name + "_externalTorqueController", registry);
      externalPositionGains.setPIDGains(1,0,0,0);
      externalTorqueGains.setPIDGains(1,0,0,0);
      externalPositionPIDController = new PIDController(externalPositionGains,"_externalPositionController",registry);
      externalTorquePIDController = new PIDController(externalTorqueGains,"_externalTorqueController",registry);
      this.DT = DT;
      this.lastMeasuredTorque = 0;

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

      //If control mode is set to EXTERNAL_PID_CONTROLLER use an external feedback signal to control motor position and velocity
      //If control mode is set to EXTERNAL_TORQUE_CONTROLLER use an external feedback signal to control motor torque
      //If control mode is set to INTERNAL_MOTOR_CONTROLLER command desired signals to TMotor
      switch(motorControlMode.getEnumValue()){
         case INTERNAL_MOTOR_CONTROLLER:
            tMotor.parseAndPack(getKp(), getKd(), desiredPosition, desiredVelocity, desiredTorque);
            tMotor.getYoCANMsg().setSent(tMotor.getControlMotorMsg().getData());
            tMotor.setCommandedMsg(tMotor.getControlMotorMsg());
            break;
         //Not much point to this case, there mostly to test PIDController logic and case logic
         case EXTERNAL_PID_CONTROLLER:
            commandedMotorSpeed.set(externalPositionPIDController.compute(measuredPosition.getDoubleValue(),
                                                  desiredMotorPosition.getDoubleValue(),
                                                  measuredVelocity.getDoubleValue(),
                                                  desiredMotorVelocity.getDoubleValue(),
                                                  this.DT
                                                  ));

            //setting position and velocity to 0 is part of TMotor Torque Command Spec
            this.setKp(0.0);
            desiredVelocity = (float) commandedMotorSpeed.getDoubleValue();
            tMotor.parseAndPack(getKp(), getKd(), 0, desiredVelocity, 0);
            tMotor.getYoCANMsg().setSent(tMotor.getControlMotorMsg().getData());
            tMotor.setCommandedMsg(tMotor.getControlMotorMsg());
            break;
         case EXTERNAL_TORQUE_CONTROLLER:
            commandedMotorTorque.set(externalTorquePIDController.compute(measuredTorque.getDoubleValue(),
                                                                         desiredMotorTorque.getDoubleValue(),
                                                                         measuredVelocity.getDoubleValue(),
                                                                         desiredMotorTorqueRate.getDoubleValue(),
                                                                         this.DT
            ));
            //setting position and velocity to 0 is part of TMotor Torque Command Spec
            desiredTorque = (float) commandedMotorTorque.getDoubleValue();
            tMotor.parseAndPack(getKp(), getKd(), 0, 0, desiredTorque);
            tMotor.getYoCANMsg().setSent(tMotor.getControlMotorMsg().getData());
            tMotor.setCommandedMsg(tMotor.getControlMotorMsg());
            break;
         default:
            throw new RuntimeException("Invalid Motor Control Mode");
      }

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

   public void updateMeasuredForce(double measuredTorque)
   {
      this.lastMeasuredTorque = this.measuredTorque.getDoubleValue();
      this.measuredTorque.set(measuredTorque);
   }

   public void updateMeasuredPosition(double measuredPosition)
   {
      this.measuredPosition.set(measuredPosition);
   }

   public void updateMeasuredVelocity(double measuredVelocity)
   {
      this.measuredVelocity.set(measuredVelocity);
   }

   public double getPositionError()
   {
      positionError.set(desiredMotorPosition.getDoubleValue() - measuredPosition.getDoubleValue());
      return positionError.getDoubleValue();
   }

   public double getVelocityError()
   {
      velocityError.set(desiredMotorVelocity.getDoubleValue() - measuredVelocity.getDoubleValue());
      return velocityError.getDoubleValue();
   }

   public double getTorqueError()
   {
      torqueError.set(desiredMotorTorque.getDoubleValue() - (measuredTorque.getDoubleValue()));
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

   public void setDesiredTorqueRate(double torqueRate)
   {
      desiredMotorTorqueRate.set(torqueRate);
   }

   public void setKp(double kp)
   {
      motorPositionKp.set((int) kp);
   }

   public void setKd(double kd)
   {
      motorVelocityKd.set((int) kd);
   }

   public void setMotorControlMode(MotorControlMode requestedControlMode){
      motorControlMode.set(requestedControlMode);
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

   public double getCommandedTorque(){return commandedMotorTorque.getDoubleValue();}


   public void sendEnableMotorCommand()
   {
      sendEnableMotorCommand.set(true);
   }

   public void sendDisableMotorCommand()
   {
      sendDisableMotorCommand.set(true);
   }
}
