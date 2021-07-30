package us.ihmc.gyemsCore;

import peak.can.basic.*;
import us.ihmc.CAN.CANMotor;
import us.ihmc.gyemsCore.CANMessages.GyemsMotorCANReceiveMessage;
import us.ihmc.gyemsCore.CANMessages.GyemsMotorCANReplyMessage;
import us.ihmc.gyemsCore.parameters.GyemsMotorParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class GyemsMotor extends CANMotor
{
    // Command messages for Gyems motor
    private final GyemsMotorCANReceiveMessage motorReceiveMsg;
    private final GyemsMotorCANReplyMessage motorReplyMsg;

    // gains
    private final YoInteger motorPositionKp;
    private final YoInteger motorPositionKi;
    private final YoInteger motorVelocityKp;
    private final YoInteger motorVelocityKi;
    private final YoInteger motorTorqueKp;
    private final YoInteger motorTorqueKi;
    private boolean requestPIDGainsUpdate = false;

    // other inputs from motor
    private final YoInteger measuredTemperature;

    // limits
//    private final YoDouble maximumActuatorSpeed = new YoDouble("maximumActuatorSpeed", registry);
//    private final YoInteger maximumMotorSpeed = new YoInteger("maximumMotorSpeed", registry);

    //desired outputs
    private final YoInteger desiredMotorPosition = new YoInteger("desiredMotorPosition", registry);

    public GyemsMotor(int ID, double dt, YoDouble time, YoRegistry parentRegistry)
    {
        super(ID, dt);

        GyemsMotorParameters encoderParameters = new GyemsMotorParameters();
        motorReceiveMsg =  new GyemsMotorCANReceiveMessage(ID, encoderParameters);
        motorReplyMsg =  new GyemsMotorCANReplyMessage(encoderParameters);

        velocityFilterCoefficient.setVariableBounds(0.0, 1.0);
        velocityFilterCoefficient.set(0.8);

        measuredTemperature = new YoInteger("measuredTemperature", registry);
        motorPositionKp = new YoInteger("motorPositionKp", registry);
        motorPositionKi = new YoInteger("motorPositionKi", registry);
        motorVelocityKp = new YoInteger("motorVelocityKp", registry);
        motorVelocityKi = new YoInteger("motorVelocityKi", registry);
        motorTorqueKp = new YoInteger("motorTorqueKp", registry);
        motorTorqueKi = new YoInteger("motorTorqueKi", registry);
        setupPIDGains(50, 30, 90, 40, 40, 20);

        desiredMotorPosition.set(0);
//        maximumMotorSpeed.set(0);    // maximumActuatorSpeed * GEAR_RATIO_TO_ONE * ENCODER_POSITION_RESOLUTION

        parentRegistry.addChild(registry);
    }

    /**
     * Set up position, velocity, and torque loop PI gains
     *
     * Data field   | Description       | Data
     * ------------------------------------------------
     * DATA[0]      | Command byte      | 0x31 for RAM, 0x32 for ROM
     * DATA[1]      | NULL              | 0x00
     * DATA[2]      | position loop kp
     * DATA[3]      | position loop ki
     * DATA[4]      | velocity loop kp
     * DATA[5]      | velocity loop ki
     * DATA[6]      | torque loop kp
     * DATA[7]      | torque loop ki
     */
    public void setupPIDGains(int positionKp, int positionKi, int velocityKp, int velocityKi, int torqueKp, int torqueKi)
    {
        byte posKp = (byte) positionKp;
        byte posKi = (byte) positionKi;
        byte velKp = (byte) velocityKp;
        byte velKi = (byte) velocityKi;
        byte tauKp = (byte) torqueKp;
        byte tauKi = (byte) torqueKi;

        motorPositionKp.set(posKp);
        motorPositionKi.set(posKi);
        motorVelocityKp.set(velKp);
        motorVelocityKi.set(velKi);
        motorTorqueKp.set(tauKp);
        motorTorqueKi.set(tauKi);

        requestPIDGainsUpdate = motorReceiveMsg.updatePIDGains(posKp, posKi, velKp, velKi, tauKp, tauKi);
    }

    public void updatePIDGains()
    {
        byte positionKp = (byte) motorPositionKp.getIntegerValue();
        byte positionKi = (byte) motorPositionKi.getIntegerValue();
        byte velocityKp = (byte) motorVelocityKp.getIntegerValue();
        byte velocityKi = (byte) motorVelocityKi.getIntegerValue();
        byte torqueKp = (byte) motorTorqueKp.getIntegerValue();
        byte torqueKi = (byte) motorTorqueKi.getIntegerValue();
        requestPIDGainsUpdate = motorReceiveMsg.updatePIDGains(positionKp, positionKi, velocityKp, velocityKi, torqueKp, torqueKi);
    }

    /**
     * Request encoder data
     */
    public TPCANMsg requestRead()
    {
        return motorReceiveMsg.getRequestReadMsg();
    }

    /**
     * Request data specified in {@code message} from motor
     * @param message  CAN message requesting access to read specific motor data
     */
    public TPCANStatus requestRead(PCANBasic can, TPCANHandle bus, TPCANMsg message)
    {
        TPCANStatus status = can.Write(bus, message);
        if(status != TPCANStatus.PCAN_ERROR_OK)
        {
            System.out.println("Unable to request CAN read message. Error: " + status.toString());
            System.exit(0);
        }
        return status;
    }

    @Override
    public void read(TPCANMsg message)
    {
        yoCANMsg.setReceived(message);

        motorReplyMsg.parseAndUnpack(message);

        measuredEncoderPosition.set(motorReplyMsg.getMeasuredEncoderPosition());
        measuredActuatorPosition.set(motorReplyMsg.getMeasuredPosition());
        measuredVelocity.set(motorReplyMsg.getMeasuredVelocity());
        measuredTorqueCurrent.set(motorReplyMsg.getMeasuredTorque());
        measuredTemperature.set(motorReplyMsg.getMeasuredTemperature());

        filteredVelocity.update();

    }

    public void update()
    {
    }

   @Override
   public TPCANMsg write()
   {
       // update PID gains IF loop
        if(requestPIDGainsUpdate)
        {
            requestPIDGainsUpdate = false;
            yoCANMsg.setSent(motorReceiveMsg.getUpdatePIDGainsMsg().getData());
            return motorReceiveMsg.getUpdatePIDGainsMsg();
        }

//       if (sendZeroMotorCommand.getBooleanValue())
//       {
//           yoCANMsg.setSent(motorReceiveMsg.getZeroMotorCommandData());
//           sendZeroMotorCommand.set(false);
//           return motorReceiveMsg.getZeroMotorMsg();
//       }

        //TODO: need to make gyems work like TMOTOR and pull information from the message
//       float desiredPosition = (float) desiredActuatorPosition.getDoubleValue();
//       float desiredVelocity = (float) desiredActuatorVelocity.getDoubleValue();
       float desiredTorque = 0.0f;

//       motorReceiveMsg.parseAndPackControlMsg(desiredPosition, desiredVelocity, desiredTorque);
       yoCANMsg.setSent(motorReceiveMsg.getControlMotorPositionMsg().getData());

       return motorReceiveMsg.getControlMotorPositionMsg();
   }
}
