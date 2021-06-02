package us.ihmc;

import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizerStateListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.*;

public class CANMotorVisualizer implements SCSVisualizerStateListener, YoVariableChangedListener
{
    private static final int DEFAULT_BUFFER_SIZE = 10000;
    private static final int DEFAULT_UPDATE_RATE = 0;

    // Data received via CAN
//    private YoInteger measuredTemperature;
    private YoInteger measuredEncoderPosition;
    private YoDouble measuredActuatorPosition;
    private YoDouble measuredVelocity;
    private YoDouble measuredTorque;
    private YoDouble measuredFilteredVelocity;
//    private YoInteger tickCounter;
    private YoInteger motorPositionKp;
//    private YoInteger motorPositionKi;
    private YoInteger motorVelocityKd;
//    private YoInteger motorVelocityKi;
//    private YoInteger motorTorqueKp;
//    private YoInteger motorTorqueKi;

//    private YoDouble maximumActuatorSpeed;
//    private YoInteger maximumMotorSpeed;

    // settings that can be changed
    private YoDouble velocityFilterCoefficient;

    // desired commands
    private YoDouble desiredActuatorPosition;
    private YoDouble desiredActuatorVelocity;

    // control settings
    private YoBoolean positionControlEnabled;

    public CANMotorVisualizer()
    {
        SCSVisualizer scsVisualizer = new SCSVisualizer(DEFAULT_BUFFER_SIZE);
        scsVisualizer.setVariableUpdateRate(DEFAULT_UPDATE_RATE);
        scsVisualizer.addSCSVisualizerStateListener(this);

        YoVariableClient client = new YoVariableClient(scsVisualizer);
        client.startWithHostSelector();
    }

    @Override
    public void starting(SimulationConstructionSet scs, Robot robot, YoRegistry registry)
    {
        LogTools.info("Starting CAN visualizer");
        scs.setFastSimulate(true);
        scs.hideViewport();

        setupYoVariables(registry);
    }

    private void setupYoVariables(YoRegistry registry)
    {

    }


    @Override
    public void changed(YoVariable source)
    {

    }

    public static void main(String[] args)
    {
        new CANMotorVisualizer();
    }

}
