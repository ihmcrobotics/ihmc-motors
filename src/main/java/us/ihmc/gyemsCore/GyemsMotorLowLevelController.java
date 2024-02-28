package us.ihmc.gyemsCore;

import peak.can.basic.TPCANMsg;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class GyemsMotorLowLevelController implements RobotController
{
   private final YoRegistry registry;
   private final GyemsMotor gyemsMotor;
   private final YoBoolean sendEnableMotorCommand;
   private final YoBoolean sendDisableMotorCommand;
   private final YoBoolean sendZeroMotorCommand;
   // gains
   private final YoInteger motorPositionKp;
   private final YoInteger motorVelocityKd;
   private final YoDouble motorTorqueKp;
   private final int CAN_SENT_BYTES = 8;
   private double unsafeOutputSpeed;
   private YoDouble desiredActuatorPosition;
   private YoDouble desiredActuatorVelocity;
   private YoDouble desiredActuatorTorque;
   private double measuredForce = 0.0;
   private double torqueError;

   public GyemsMotorLowLevelController(String name, GyemsMotor gyemsMotor, YoRegistry parentRegistry)
   {
      this.gyemsMotor = gyemsMotor;
      this.registry = new YoRegistry(getClass().getSimpleName() + "_" + name);

      desiredActuatorPosition = new YoDouble(name + "_desiredActuatorPosition", registry);
      desiredActuatorVelocity = new YoDouble(name + "_desiredActuatorVelocity", registry);
      desiredActuatorTorque = new YoDouble(name + "_desiredActuatorTorque", registry);

      sendEnableMotorCommand = new YoBoolean(name + "_sendEnableMotorCommand", registry);
      sendDisableMotorCommand = new YoBoolean(name + "_sendDisableMotorCommand", registry);
      sendZeroMotorCommand = new YoBoolean(name + "_sendZeroMotorCommand", registry);

      motorPositionKp = new YoInteger(name + "_motorPositionKp", registry);
      motorVelocityKd = new YoInteger(name + "_motorVelocityKd", registry);
      motorTorqueKp = new YoDouble(name + "_motorTorqueKp", registry);

      parentRegistry.addChild(registry);
   }

   /**
    * Computes command to send to the Gyems Motor
    */
   @Override
   public void doControl()
   {
      if (isUserSendingPredefinedCommand() || isMotorInUnsafeState())
         return;

      float desiredPosition = (float) desiredActuatorPosition.getDoubleValue();
      float desiredVelocity = (float) desiredActuatorVelocity.getDoubleValue();
      float desiredTorque = (float) desiredActuatorTorque.getDoubleValue();

      gyemsMotor.parseAndPack(desiredPosition, desiredVelocity, desiredTorque);
      gyemsMotor.getYoCANMsg().setSent(gyemsMotor.getControlMotorMsg().getData(), CAN_SENT_BYTES);
      gyemsMotor.setCommandedMsg(gyemsMotor.getControlMotorMsg());
   }

   private boolean isUserSendingPredefinedCommand()
   {
      if (gyemsMotor.isRequestingPIDGainsUpdate())
      {   //TODO see if this can be done along with the control command so that
         // we don't spend a whole tick just to change the PID gains
         gyemsMotor.setCommandedMsg(gyemsMotor.getUpdatedPIDGainsMsg());
         gyemsMotor.getYoCANMsg().setSent(gyemsMotor.getUpdatedPIDGainsMsg().getData(), CAN_SENT_BYTES);
         gyemsMotor.setRequestPIDGainsUpdate(false);
         return true;
      }
      return false;
   }

   private boolean isMotorInUnsafeState()
   {
      //        if(motorIsInUnsafeState())
      //        {
      //            //TODO implement
      //            gyemsMotor.setCommandedMsg( gyemsMotor.getStopMotorMsg() );
      //            gyemsMotor.getYoCANMsg().setSent(gyemsMotor.getStopMotorMsg().getData());
      //            return true;
      //        }
      return false;
   }

   private boolean motorIsInUnsafeState()
   {
      if (Math.abs(gyemsMotor.getVelocity()) > unsafeOutputSpeed)
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
      gyemsMotor.read(receivedMsg);
   }

   public void update()
   {
      gyemsMotor.update();
   }

   public void updateMeasuredForce(double measuredForce)
   {
      this.measuredForce = measuredForce;
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

   public GyemsMotor getMotor()
   {
      return this.gyemsMotor;
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

   public void setTorqueError(double torqueError)
   {
      this.torqueError = torqueError;
   }
}