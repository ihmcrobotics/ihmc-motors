package us.ihmc.tMotorCore;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class TMotorController implements RobotController
{
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

    protected double desiredActuatorPosition;
    protected double desiredActuatorVelocity;
    protected double desiredActuatorTorque;

    public TMotorController(String name, YoRegistry parentRegistry)
    {

        parentRegistry.addChild(registry);
    }
    @Override
    public void doControl()
    {

    }

    @Override
    public void initialize()
    {

    }

    @Override
    public YoRegistry getYoRegistry() {
        return registry;
    }

    public void setDesireds(double position, double velocity, double torque)
    {
        desiredActuatorPosition = position;
        desiredActuatorVelocity = velocity;
        desiredActuatorTorque = torque;
    }
}
