package us.ihmc.sensors.LoadStarILoad.testbed;

import us.ihmc.commons.Conversions;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

import java.io.IOException;

public class LoadStarTestBedEtherCAT extends RealtimeThread
{
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
    private final YoVariableServer yoVariableServer;

    private static final double DT = 0.001;
    private static final boolean USE_DC = false;
    private static final int priority = PriorityParameters.getMaximumPriority();
    private static final PriorityParameters controllerPriority = new PriorityParameters(priority - 5);

    private final YoInteger workingCounterMismatch = new YoInteger("workingCounterMismatch", registry);
    private final YoInteger lastWorkingCounterExpected = new YoInteger("lastWorkingCounterExpected", registry);
    private final YoInteger lastWorkingCounterActual = new YoInteger("lastWorkingCounterActual", registry);
    private final YoInteger missedDeadlinesNumber = new YoInteger("missedDeadlinesNumber", registry);
    private final YoInteger datagramLost = new YoInteger("datagramLost", registry);

    private final YoLong jitterEstimate = new YoLong("jitterEstimate", registry);
    private final YoLong lastCycleDuration = new YoLong("lastCycleDuration", registry);
    private final YoLong lastEtherCATTransactionTime = new YoLong("lastEtherCATTransactionTime", registry);
    private final YoLong lastIdleTime = new YoLong("lastIdleTime", registry);
    private final YoLong dcOffsetError = new YoLong("dcOffsetError", registry);
    private final YoLong controllerTimeInNanos = new YoLong("controllerTimeInNanos", registry);
    private final YoDouble controllerTimeInSeconds = new YoDouble("controllerTimeInSeconds", registry);

    private final YoBoolean readingOn = new YoBoolean("readingOn", registry);
    private final YoDouble measuredForceN = new YoDouble("measuredForceN", registry);
    private final YoDouble measuredForceLb = new YoDouble("measuredForceLb", registry);

    public LoadStarTestBedEtherCAT(String iface, MonotonicTime period, boolean enableDC, YoVariableServer yoVariableServer) throws IOException
    {
        super(controllerPriority);
        this.yoVariableServer = yoVariableServer;
        YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
        yoVariableServer.setMainRegistry(registry, yoGraphicsListRegistry);

        yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);

        new DefaultParameterReader().readParametersInRegistry(registry);
    }

    private void read()
    {
    }

    public void setRead(boolean flag)
    {
        readingOn.set(flag);
    }

    public static void main(String[] args) throws IOException
    {
        String iface = args[0];
        MonotonicTime period = new MonotonicTime(0, Conversions.secondsToNanoseconds(DT));
        YoVariableServer yoVariableServer = new YoVariableServer(LoadStarTestBed.class, null, new DataServerSettings(false), DT);
        LoadStarTestBedEtherCAT testbed = new LoadStarTestBedEtherCAT(iface, period, USE_DC, yoVariableServer);
        testbed.setRead(true);

        yoVariableServer.start();

        testbed.start();
        testbed.join();

        yoVariableServer.close();
    }

    protected void doControl()
    {
//        controllerTimeInNanos.set(System.nanoTime() - initialTime);
//        controllerTimeInSeconds.set(Conversions.nanosecondsToSeconds(getCurrentCycleTimestamp() - getInitTimestamp()));
    }

    protected void doReporting() {

        //skip here if needed
        yoVariableServer.update(controllerTimeInNanos.getLongValue());
    }

    protected void datagramLost() {
        this.datagramLost.increment();
    }
}
