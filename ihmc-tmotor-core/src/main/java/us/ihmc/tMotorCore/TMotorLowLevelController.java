package us.ihmc.tMotorCore;

import us.ihmc.sensors.TorqueToForceTransmission;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class TMotorLowLevelController implements RobotController
{
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

    public enum MotorControlMode{POSITION, TORQUE}
    private final YoEnum<MotorControlMode> controlMode;

    private double desiredActuatorPosition;
    private double desiredActuatorVelocity;
    private double desiredActuatorTorque;

    private double measuredForce;
    private final TorqueToForceTransmission torqueToForce;

    // gains
    private final YoInteger motorPositionKp;
    private final YoInteger motorVelocityKd;
    private final YoDouble motorTorqueKp;

    public TMotorLowLevelController(String name, YoRegistry parentRegistry)
    {
        controlMode = new YoEnum<>(name + "controlMode", registry, MotorControlMode.class);
        controlMode.set(MotorControlMode.POSITION);

        motorPositionKp = new YoInteger(name + "motorPositionKp", registry);
        motorVelocityKd = new YoInteger(name + "motorVelocityKd", registry);
        motorTorqueKp = new YoDouble(name + "motorTorqueKp", registry);

        torqueToForce = new TorqueToForceTransmission(0.05, registry);

        parentRegistry.addChild(registry);
    }

    /**
     * Adds any additional terms that are not included in the T-Motor CAN
     * messages, e.g., through the feedforward torque
     */
    @Override
    public void doControl()
    {
        double forceError = torqueToForce.getDesiredForce() - measuredForce;
        desiredActuatorTorque += motorTorqueKp.getDoubleValue() * (forceError * torqueToForce.getMotorPulleyRadius());
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public YoRegistry getYoRegistry() {
        return registry;
    }

    public void setDesireds(double torque)
    {
        setDesireds(torque, 0.0);
    }

    public void setDesireds(double position, double velocity)
    {
        switch (controlMode.getEnumValue())
        {
            case POSITION:
                desiredActuatorPosition = position;
                desiredActuatorVelocity = velocity;
                desiredActuatorTorque = 0.0;
                break;

            case TORQUE:
                desiredActuatorPosition = 0.0;
                desiredActuatorVelocity = 0.0;
                desiredActuatorTorque = position;
                torqueToForce.update(desiredActuatorTorque);
                break;
            default:
                desiredActuatorPosition = 0.0;
                desiredActuatorVelocity = 0.0;
                desiredActuatorTorque = 0.0;
                break;
        }
    }

    public void setMeasuredForce(double measuredForce)
    {
        this.measuredForce = measuredForce;
    }

    public double getDesiredPosition()
    {
        return desiredActuatorPosition;
    }

    public double getDesiredVelocity()
    {
        return desiredActuatorVelocity;
    }

    public double getDesiredTorque()
    {
        return desiredActuatorTorque;
    }

    public int getKp()
    {
        return motorPositionKp.getIntegerValue();
    }

    public int getKd()
    {
        return motorVelocityKd.getIntegerValue();
    }
}
