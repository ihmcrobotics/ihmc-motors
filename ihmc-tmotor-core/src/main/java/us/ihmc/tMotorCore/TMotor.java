package us.ihmc.tMotorCore;

import peak.can.basic.PCANBasic;
import peak.can.basic.TPCANHandle;
import peak.can.basic.TPCANMsg;
import us.ihmc.CAN.CANMotor;
import us.ihmc.sensors.TorqueToForceTransmission;
import us.ihmc.tMotorCore.CANMessages.TMotorCANReceiveMessage;
import us.ihmc.tMotorCore.CANMessages.TMotorCANReplyMessage;
import us.ihmc.tMotorCore.parameters.TMotorParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class TMotor extends CANMotor
{
   private final TMotorCANReceiveMessage motorReceiveMsg;      // CAN message sent to motor
   private final TMotorCANReplyMessage motorReplyMsg;          // CAN message reply from motor
   private TPCANMsg commandedMsg;

   private final YoBoolean startTrajectory = new YoBoolean("startTrajectory", registry);
   private final YoBoolean firstTimeInWalking = new YoBoolean("firstTimeInWalking", registry);
   private boolean durationHasChanged = false;

   private int kp;
   private int kd;
   private float desiredPosition;
   private float desiredVelocity;
   private float desiredTorque;

   private final YoDouble loadtestWeight = new YoDouble("loadtestWeight", registry);
   private final YoDouble percentGait = new YoDouble("percentGait", registry);

   private final YoDouble motorDirection = new YoDouble("motorDirection", registry);


   public TMotor(int ID, TMotorVersion version, double dt, YoRegistry parentRegistry)
   {
      super(ID, dt);
      String prefix = ID + "_";

      TMotorParameters encoderParameters = version.getMotorParameters();
      motorReceiveMsg =  new TMotorCANReceiveMessage(ID, encoderParameters);
      motorReplyMsg = new TMotorCANReplyMessage(encoderParameters);

      velocityFilterCoefficient.setVariableBounds(0.0, 1.0);
      velocityFilterCoefficient.set(0.9);

      motorDirection.set(1);
      firstTimeInWalking.set(true);

      parentRegistry.addChild(registry);
   }

   @Override
   public void read(TPCANMsg message)
   {
      yoCANMsg.setReceived(message);

      motorReplyMsg.parseAndUnpack(message);

      measuredActuatorPosition.set(motorDirection.getValue() * motorReplyMsg.getMeasuredPosition());
      measuredVelocity.set(motorDirection.getValue() * motorReplyMsg.getMeasuredVelocity());
      measuredTorqueCurrent.set(motorDirection.getValue() * motorReplyMsg.getMeasuredTorque());

      filteredVelocity.update();
   }

   public void update()
   {

   }

   @Override
   public TPCANMsg write()
   {
      parseAndPack(kp, kd, desiredPosition, desiredVelocity, desiredTorque);
      yoCANMsg.setSent(motorReceiveMsg.getControlMotorCommandData());

      return motorReceiveMsg.getControlMotorMsg();
   }

   @Override
   public void setCanBus(PCANBasic canBus, TPCANHandle channel)
   {
      this.canBus = canBus;
      this.channel = channel;
   }

   public void parseAndPack(int kp, int kd, float desiredPosition, float desiredVelocity, float desiredTorque)
   {
      motorReceiveMsg.parseAndPackControlMsg((float)motorDirection.getValue() * desiredPosition,
                                             (float)motorDirection.getValue() * desiredVelocity,
                                              (float)motorDirection.getValue() * desiredTorque,
                                                            kp, kd);
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

   public int getID()
   {
      return ID;
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
