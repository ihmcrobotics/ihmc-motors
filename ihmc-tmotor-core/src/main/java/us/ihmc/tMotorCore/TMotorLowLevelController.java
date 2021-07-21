package us.ihmc.tMotorCore;

import peak.can.basic.TPCANMsg;
import us.ihmc.sensors.TorqueToForceTransmission;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class TMotorLowLevelController implements RobotController
{
    private final YoRegistry registry;

    private double unsafeOutputSpeed;

    private YoDouble desiredActuatorPosition;
    private YoDouble desiredActuatorVelocity;
    private YoDouble desiredActuatorTorque;

    private final TMotor tMotor;

    private final TorqueToForceTransmission torqueToForce;
    private double measuredForce = 0.0;

    private final YoBoolean sendEnableMotorCommand;
    private final YoBoolean sendDisableMotorCommand;
    private final YoBoolean sendZeroMotorCommand;

    // gains
    private final YoInteger motorPositionKp;
    private final YoInteger motorVelocityKd;
    private final YoDouble motorTorqueKp;



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

        motorPositionKp = new YoInteger(name + "_motorPositionKp", registry);
        motorVelocityKd = new YoInteger(name + "_motorVelocityKd", registry);
        motorTorqueKp = new YoDouble(name + "_motorTorqueKp", registry);

        torqueToForce = new TorqueToForceTransmission(0.05, name, registry);

        parentRegistry.addChild(registry);
    }

    /**
     * Computes command to send to T-Motor
     */
    @Override
    public void doControl()
    {
        if(isUserSendingPredefinedCommand() || isMotorInUnsafeState())
            return;

        float desiredPosition = (float) desiredActuatorPosition.getDoubleValue();
        float desiredVelocity = (float) desiredActuatorVelocity.getDoubleValue();
        float desiredTorque = (float) desiredActuatorTorque.getDoubleValue();

        double forceError = torqueToForce.getDesiredForce() - measuredForce;
        desiredTorque += motorTorqueKp.getDoubleValue() * (forceError * torqueToForce.getMotorPulleyRadius());

        tMotor.parseAndPack(getKp(), getKd(), desiredPosition, desiredVelocity, desiredTorque);
        tMotor.getYoCANMsg().setSent(tMotor.getControlMotorMsg().getData());
        tMotor.setCommandedMsg(tMotor.getControlMotorMsg());
    }

    private boolean isUserSendingPredefinedCommand()
    {
        if (sendEnableMotorCommand.getBooleanValue())
        {
            tMotor.setCommandedMsg( tMotor.getEnableMotorMsg() );
            tMotor.getYoCANMsg().setSent(tMotor.getEnableMotorMsg().getData());
            sendEnableMotorCommand.set(false);
            return true;
        }

        if (sendDisableMotorCommand.getBooleanValue())
        {
            tMotor.setCommandedMsg(  tMotor.getDisableMotorMsg() );
            tMotor.getYoCANMsg().setSent(tMotor.getDisableMotorMsg().getData());
            sendDisableMotorCommand.set(false);
            return true;
        }

        if (sendZeroMotorCommand.getBooleanValue())
        {
            tMotor.setCommandedMsg( tMotor.getZeroMotorMsg() );
            tMotor.getYoCANMsg().setSent(tMotor.getZeroMotorMsg().getData());
            sendZeroMotorCommand.set(false);
            return true;
        }
        return false;
    }

    private boolean isMotorInUnsafeState()
    {
        if(motorIsInUnsafeState())
        {
            tMotor.setCommandedMsg( tMotor.getDisableMotorMsg() );
            tMotor.getYoCANMsg().setSent(tMotor.getDisableMotorMsg().getData());
            return true;
        }
        return false;
    }

    private boolean motorIsInUnsafeState()
    {
        if( Math.abs(tMotor.getVelocity()) > unsafeOutputSpeed)
            return true;

        return false;
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public YoRegistry getYoRegistry() {
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

    public void updateTorqueToForce(double tau_d) {
        torqueToForce.update(tau_d);
    }

    public void updateMeasuredForce(double measuredForce) {this.measuredForce = measuredForce;}

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

    public TMotor getMotor(){return this.tMotor;}

    public double getMeasuredAppliedTorque()
    {
        return tMotor.getTorque();
    }

    public double getMeasuredPosition() {return tMotor.getPosition();}

    public double getMeasuredVelocity() {return tMotor.getVelocity();}

    public double getMeasuredTorque() {return tMotor.getTorque();}

    public int getKp()
    {
        return motorPositionKp.getIntegerValue();
    }

    public int getKd()
    {
        return motorVelocityKd.getIntegerValue();
    }

    public double getTorqueKp(){return this.motorTorqueKp.getDoubleValue();}
}
