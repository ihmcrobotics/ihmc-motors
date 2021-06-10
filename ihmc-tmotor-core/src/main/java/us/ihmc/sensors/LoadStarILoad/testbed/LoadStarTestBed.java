package us.ihmc.sensors.LoadStarILoad.testbed;

import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.sensors.LoadStarILoad.LoadStarILoad;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.io.IOException;

public class LoadStarTestBed implements Runnable
{
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
    private final YoVariableServer yoVariableServer;

    private static final double DT = 0.001;
    private static final String COM_PORT = "COM9";
    private LoadStarILoad load;

    private final YoDouble measuredForce = new YoDouble("measuredForce", registry);
    private final YoBoolean readingOn = new YoBoolean("readingOn", registry);

    public LoadStarTestBed(YoVariableServer yoVariableServer)
    {
        this.yoVariableServer = yoVariableServer;
        yoVariableServer.setMainRegistry(registry, null);
    }

    private void start() throws IOException
    {
        load = new LoadStarILoad(COM_PORT);
    }

    private void read()
    {
        load.outputWeightOnce();
        measuredForce.set(load.getForceNewton());
    }

    public static void main(String[] args) throws IOException
    {
        YoVariableServer yoVariableServer = new YoVariableServer(LoadStarTestBed.class, null, new DataServerSettings(false), DT);
        LoadStarTestBed testbed = new LoadStarTestBed(yoVariableServer);
        yoVariableServer.start();

        testbed.start();

        yoVariableServer.close();

    }

    @Override
    public void run()
    {
        while (readingOn.getBooleanValue())
        {
            read();
        }
    }
}
