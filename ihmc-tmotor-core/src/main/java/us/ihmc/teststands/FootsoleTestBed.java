package us.ihmc.teststands;

import us.ihmc.commons.Conversions;
import us.ihmc.etherCAT.master.EtherCATRealtimeThread;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.process.LinuxProcess;
import us.ihmc.process.Scheduler;
import us.ihmc.process.SchedulerAlgorithm;
import us.ihmc.realtime.CPUDMALatency;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.sensors.footsole.EtherCATPressureSensor;
import us.ihmc.sensors.footsole.EvaFootsole;
import us.ihmc.sensors.footsole.YoEtherCATPressureSensor;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

import java.io.IOException;
import java.util.List;

public class FootsoleTestBed extends EtherCATRealtimeThread
{
    private static final boolean USE_DC = false;
    private static final double DT = 0.001;
    private final static PriorityParameters controllerPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);

    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
    private final YoVariableServer yoVariableServer;

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

    private final YoLong readErrorCounter = new YoLong("readErrorCounter", registry);
    private final YoLong writeErrorCounter = new YoLong("writeErrorCounter", registry);

    // lan slave
    private final YoEtherCATPressureSensor yoPressureArraySensor;
    private final EtherCATPressureSensor lan9252Slave;
    private final EvaFootsole footsole;

    public FootsoleTestBed(String iface, MonotonicTime period, boolean enableDC, YoVariableServer yoVariableServer) throws IOException
    {
        super(iface, controllerPriority, period, enableDC, 250000);
        LogTools.info("here " + System.getProperty("user.dir"));
        setEtherCATPriorityForInterface(iface);
        CPUDMALatency.setLatency(0);

        YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
        this.yoVariableServer = yoVariableServer;

        lan9252Slave = new EtherCATPressureSensor(0, 0);
        yoPressureArraySensor = new YoEtherCATPressureSensor(lan9252Slave, registry);
        yoPressureArraySensor.setCoeffs(0.0, 1.0, 0.0);

        yoVariableServer.setMainRegistry(registry, yoGraphicsListRegistry);
//        yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);

        double footLength = 0.28;
        double footWidth = 0.12;
        footsole = new EvaFootsole(footLength, footWidth, registry, yoGraphicsListRegistry);

        registerSlave(lan9252Slave);
    }

    private void initialize()
    {

    }

    private void motorRead()
    {

    }

    private void motorWrite()
    {

    }

    private void setEtherCATPriorityForInterface(String iface)
    {
        try
        {
            List<LinuxProcess> ethernetIRQThreads = LinuxProcess.getProcessesByPattern("irq/\\d+-" + iface + ".*");
            for (LinuxProcess ethernetIRQThread : ethernetIRQThreads)
            {
                Scheduler.setScheduler(ethernetIRQThread, SchedulerAlgorithm.SCHED_FIFO, 90);
            }
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    /**
     * Callback to notify controller that there was a difference in working counter This gets called
     * when the expected working counter and actual working counter differ. It is recommended to go to a
     * safe state when this function gets called. This function gets called before the state of the
     * slaves is updated. All slaves will probably be in OP mode till the EtherCAT householder thread
     * checks the state. This can take 10ms or more.
     *
     * @param expected
     * @param actual
     */
    @Override
    protected void workingCounterMismatch(int expected, int actual)
    {
        this.workingCounterMismatch.increment();
        this.lastWorkingCounterExpected.set(expected);
        this.lastWorkingCounterActual.set(actual);
    }

    /**
     * Callback to notify controller of missed deadline
     */
    @Override
    protected void deadlineMissed()
    {
        this.missedDeadlinesNumber.increment();
    }

    /**
     * Callback called cyclically to do the control loop. Will not get called till all actuators are
     * online. Will also not get called when a deadline is missed or a datagram got lost.
     */
    @Override
    protected void doControl()
    {
        controllerTimeInNanos.set(getCurrentCycleTimestamp() - getInitTimestamp());
        controllerTimeInSeconds.set(Conversions.nanosecondsToSeconds(getCurrentCycleTimestamp() - getInitTimestamp()));

        yoPressureArraySensor.read();
        footsole.updateCoP(yoPressureArraySensor.getMeasuredNormalForces());

        motorRead();

        motorWrite();
    }

    /**
     * Callback called cyclically to do reporting and logging. Will always get called, even when the
     * controller is not yet online or a deadline is missed. Make sure this function returns quickly.
     */
    @Override
    protected void doReporting()
    {
        jitterEstimate.set(getJitterEstimate());
        lastCycleDuration.set(getLastCycleDuration());
        lastEtherCATTransactionTime.set(getEtherCATTransactionTime());
        lastIdleTime.set(getIdleTime());
        dcOffsetError.set(getDCOffsetError());

        //skip here if needed
        yoVariableServer.update(controllerTimeInNanos.getLongValue());
    }

    /**
     * The receive function of the master timed out and no packet was received. This means a packet got
     * corrupted or dropped on the slave network and is generally bad news.
     */
    @Override
    protected void datagramLost()
    {
        this.datagramLost.increment();
    }

    public static void main(String[] args) throws IOException
    {
        String iface = args[0];
        MonotonicTime period = new MonotonicTime(0, Conversions.secondsToNanoseconds(DT));

        YoVariableServer yoVariableServer = new YoVariableServer(FootsoleTestBed.class, null, new DataServerSettings(true), period.asSeconds());

        FootsoleTestBed testbed = new FootsoleTestBed(iface, period, USE_DC, yoVariableServer);
        testbed.initialize();

        yoVariableServer.start();
        testbed.start();

        testbed.join();
        yoVariableServer.close();
    }


}
