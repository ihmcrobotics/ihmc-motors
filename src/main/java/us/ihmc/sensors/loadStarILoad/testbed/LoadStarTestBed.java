package us.ihmc.sensors.loadStarILoad.testbed;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.realtime.CPUDMALatency;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.sensors.loadStarILoad.serial.SerialLoadcell;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class LoadStarTestBed extends RealtimeThread
{
   private static final double DT = 0.001;
   private static final MonotonicTime period = new MonotonicTime(0, Conversions.secondsToNanoseconds(DT));
   private final static PriorityParameters controllerPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);
   // serial communication
   private static final String COMPort = "/dev/ttyUSB0";   // or "COM10" on exo OCU; /dev/ttyACM0
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer;
   private final SerialLoadcell serial = new SerialLoadcell(COMPort);
   private final YoDouble measuredForceN = new YoDouble("measuredForceN", registry);
   //    private final YoBoolean readEnabled = new YoBoolean("readEnabled", registry);
   private final YoDouble measuredForceLb = new YoDouble("measuredForceLb", registry);
   private boolean readOnLastTick = true;
   private YoLong controllerTimeInNanos = new YoLong("controllerTimeInNanos", registry);

   public LoadStarTestBed(YoVariableServer yoVariableServer)
   {
      super(controllerPriority, new PeriodicParameters(period));

      CPUDMALatency.setLatency(0);

      this.yoVariableServer = yoVariableServer;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoVariableServer.setMainRegistry(registry, yoGraphicsListRegistry);

      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);

      startSerialConnection();
      ThreadTools.sleep(1000);

      new DefaultParameterReader().readParametersInRegistry(registry);
   }

   //    private void initialize()
   //    {
   //        readEnabled.set(true);
   //    }

   public static void main(String[] args) throws InterruptedException
   {
      YoVariableServer yoVariableServer = new YoVariableServer(LoadStarTestBed.class, null, new DataServerSettings(false), DT);
      LoadStarTestBed testbed = new LoadStarTestBed(yoVariableServer);
      //        testbed.initialize();

      yoVariableServer.start();
      //        Thread thread = new Thread(testbed);

      testbed.start();
      testbed.join();

      yoVariableServer.close();
   }

   private void startSerialConnection()
   {
      serial.connect();
   }

   private void read()
   {
      if (readOnLastTick)
      {
         serial.outputWeightOnce();
         measuredForceN.set(serial.readForce());
      }
      readOnLastTick = !readOnLastTick;
   }

   private void doControl()
   {
   }

   private void write()
   {
      //        if(readOnLastTick)
      //        {
      //            serial.outputWeightOnce();
      //        }

   }

   @Override
   public void run()
   {
      while (true)
      {
         ThreadTools.sleep(1);

         read();
         doControl();
         write();

         controllerTimeInNanos.set(System.nanoTime());
         yoVariableServer.update(controllerTimeInNanos.getLongValue());
      }
   }
}
