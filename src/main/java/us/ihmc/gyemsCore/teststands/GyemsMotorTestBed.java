package us.ihmc.gyemsCore.teststands;

import gnu.trove.map.hash.TIntObjectHashMap;
import peak.can.basic.PCANBasic;
import peak.can.basic.TPCANBaudrate;
import peak.can.basic.TPCANHandle;
import peak.can.basic.TPCANMsg;
import peak.can.basic.TPCANStatus;
import peak.can.basic.TPCANType;
import us.ihmc.commons.Conversions;
import us.ihmc.gyemsCore.GyemsMotor;
import us.ihmc.gyemsCore.GyemsMotorLowLevelController;
import us.ihmc.gyemsCore.canMessages.GyemsMotorCANReplyMessage;
import us.ihmc.realtime.CPUDMALatency;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.HashMap;

import static peak.can.basic.TPCANStatus.PCAN_ERROR_QRCVEMPTY;

public class GyemsMotorTestBed extends RealtimeThread
{
   private static final double DT = 0.001;
   private static final MonotonicTime period = new MonotonicTime(0, Conversions.secondsToNanoseconds(DT));
   private final static PriorityParameters controllerPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);
   private static final int RMDX8_CAN_ID = 0x141;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer;
   private final YoLong tickStartTimeInNanos = new YoLong("tickStartTimeInNanos", registry);
   private final YoLong doControlTime = new YoLong("doControlTime", registry);
   private final YoLong computeTime = new YoLong("computeTime", registry);
   private final YoLong canReadTime = new YoLong("canReadTime", registry);
   private final YoLong canWriteTime = new YoLong("canWriteTime", registry);
   private final YoLong effectiveDT = new YoLong("effectiveDT", registry);
   private final YoDouble yoTime = new YoDouble("yoTime", registry);
   private final YoLong readErrorCounter = new YoLong("readErrorCounter", registry);
   private final YoLong writeErrorCounter = new YoLong("writeErrorCounter", registry);
   // motors in CAN bus
   private final TIntObjectHashMap<GyemsMotor> motors = new TIntObjectHashMap<>();
   private final int[] motorIDs;
   private final HashMap<GyemsMotor, GyemsMotorLowLevelController> motorControllers = new HashMap<>();
   private final YoFunctionGenerator functionGenerator;
   private final TPCANHandle channel = TPCANHandle.PCAN_PCIBUS1;
   private final TPCANMsg receivedMsg = new TPCANMsg();
   // specialized YoVariables
   private final YoBoolean enableCANMsgs = new YoBoolean("enableCANMsgs", registry);
   private final YoBoolean resetCounters = new YoBoolean("resetCounters", registry);
   //debug
   private final YoBoolean justRead = new YoBoolean("justRead", registry);
   private final YoInteger messagesInReadBus = new YoInteger("messagesInBus", registry);
   // CAN-related goodies
   private PCANBasic can = new PCANBasic();
   private TPCANStatus status = null;
   private YoInteger motorPositionKp;
   private YoInteger motorPositionKi;
   private YoInteger motorVelocityKp;
   private YoInteger motorVelocityKi;
   private YoInteger motorTorqueKp;
   private YoInteger motorTorqueKi;

   public GyemsMotorTestBed(YoVariableServer yoVariableServer)
   {
      super(controllerPriority, new PeriodicParameters(period));

      CPUDMALatency.setLatency(0);

      this.yoVariableServer = yoVariableServer;
      yoVariableServer.setMainRegistry(registry, null);

      GyemsMotor gyemsMotor = new GyemsMotor(RMDX8_CAN_ID, "GyemsMotor", DT, registry);
      motors.put(gyemsMotor.getID(), gyemsMotor);
      GyemsMotorLowLevelController gyemsController = new GyemsMotorLowLevelController("gyemsController", gyemsMotor, registry);
      gyemsController.setUnsafeOutputSpeed(16.0);
      motorControllers.put(gyemsMotor, gyemsController);
      functionGenerator = new YoFunctionGenerator("functionGenerator", yoTime, registry);
      functionGenerator.setAlphaForSmoothing(0.99);

      motorIDs = motors.keys();

      justRead.set(false);
      receivedMsg.setLength((byte) 6);
      enableCANMsgs.set(true);
   }

   public static void main(String[] args)
   {
      YoVariableServer yoVariableServer = new YoVariableServer(GyemsMotorTestBed.class, null, new DataServerSettings(true), DT);
      GyemsMotorTestBed testbed = new GyemsMotorTestBed(yoVariableServer);

      testbed.initialize();

      yoVariableServer.start();

      testbed.start();
      testbed.join();

      yoVariableServer.close();
   }

   private void initialize()
   {
      if (!can.initializeAPI())
      {
         System.out.println("Unable to initialize the API");
         System.exit(0);
      }
      else
      {
         System.out.println("CAN API has been initialized");
      }
      status = can.Initialize(channel, TPCANBaudrate.PCAN_BAUD_1M, TPCANType.PCAN_TYPE_NONE, 0, (short) 0);
      //     can.SetRcvEvent(channel);
      //      motor.sendPIDGainsToController(can, channel);
      initializeYoVariables();
   }

   private void initializeYoVariables()
   {
      motorPositionKp = (YoInteger) registry.findVariable("motorPositionKp");
      motorPositionKi = (YoInteger) registry.findVariable("motorPositionKi");
      motorVelocityKp = (YoInteger) registry.findVariable("motorVelocityKp");
      motorVelocityKi = (YoInteger) registry.findVariable("motorVelocityKi");
      motorTorqueKp = (YoInteger) registry.findVariable("motorTorqueKp");
      motorTorqueKi = (YoInteger) registry.findVariable("motorTorqueKi");

      motorPositionKp.addListener(e ->
                                  {
                                     for (int i = 0; i < motorIDs.length; i++)
                                        motors.get(motorIDs[i]).updatePIDGains();
                                  });
      motorPositionKi.addListener(e ->
                                  {
                                     for (int i = 0; i < motorIDs.length; i++)
                                        motors.get(motorIDs[i]).updatePIDGains();
                                  });
      motorVelocityKp.addListener(e ->
                                  {
                                     for (int i = 0; i < motorIDs.length; i++)
                                        motors.get(motorIDs[i]).updatePIDGains();
                                  });
      motorVelocityKi.addListener(e ->
                                  {
                                     for (int i = 0; i < motorIDs.length; i++)
                                        motors.get(motorIDs[i]).updatePIDGains();
                                  });
      motorTorqueKp.addListener(e ->
                                {
                                   for (int i = 0; i < motorIDs.length; i++)
                                      motors.get(motorIDs[i]).updatePIDGains();
                                });
      motorTorqueKi.addListener(e ->
                                {
                                   for (int i = 0; i < motorIDs.length; i++)
                                      motors.get(motorIDs[i]).updatePIDGains();
                                });
   }

   @Override
   public void run()
   {
      long controllerStartTime = System.nanoTime();
      while (true)
      {
         effectiveDT.set(System.nanoTime() - tickStartTimeInNanos.getLongValue());
         tickStartTimeInNanos.set(System.nanoTime());

         yoTime.set(Conversions.nanosecondsToSeconds(tickStartTimeInNanos.getLongValue() - controllerStartTime));

         if (resetCounters.getBooleanValue())
         {
            readErrorCounter.set(0);
            writeErrorCounter.set(0);
            resetCounters.set(false);
         }

         if (enableCANMsgs.getBooleanValue())
         {

            long canReadStartTime = System.nanoTime();
            read();
            canReadTime.set(System.nanoTime() - canReadStartTime);

            long computeStartTime = System.nanoTime();
            compute();
            computeTime.set(System.nanoTime() - computeStartTime);

            long canWriteStartTime = System.nanoTime();
            write();
            canWriteTime.set(System.nanoTime() - canWriteStartTime);

            // TODO safe sentinel
         }
         // wait to free sent queue
         yoVariableServer.update(tickStartTimeInNanos.getLongValue());
         doControlTime.set(System.nanoTime() - tickStartTimeInNanos.getLongValue());
         waitForNextPeriod();
      }
   }

   private void read()
   {
      TPCANStatus readStatus = can.Read(channel, receivedMsg, null);

      messagesInReadBus.set(0);
      while (readStatus != PCAN_ERROR_QRCVEMPTY)
      {
         if (readStatus == TPCANStatus.PCAN_ERROR_OK)
         {
            int id = GyemsMotorCANReplyMessage.getID(receivedMsg);
            if (motors.containsKey(id))
               motors.get(id).read(receivedMsg);
            messagesInReadBus.increment();
         }
         else
         {
            readErrorCounter.increment();
         }
         readStatus = can.Read(channel, receivedMsg, null);
      }
   }

   private void compute()
   {
      for (int id = 0; id < motorIDs.length; id++)
      {
         motorControllers.get(motors.get(motorIDs[id])).setDesiredPosition(functionGenerator.getValue());
         motorControllers.get(motors.get(motorIDs[id])).setDesiredVelocity(functionGenerator.getValueDot());
         motorControllers.get(motors.get(motorIDs[id])).setDesiredTorque(0.0);

         motorControllers.get(motors.get(motorIDs[id])).doControl();
      }
   }

   private void write()
   {
      if (justRead.getBooleanValue())
      {
         for (int id = 0; id < motorIDs.length; id++)
         {
            TPCANMsg motorCommand = motors.get(motorIDs[id]).requestRead();
            status = can.Write(channel, motorCommand);
         }
      }
      else
      {
         for (int id = 0; id < motorIDs.length; id++)
         {
            TPCANMsg motorCommand = motors.get(motorIDs[id]).getCommandedMsg();
            status = can.Write(channel, motorCommand);
            if (status != TPCANStatus.PCAN_ERROR_OK)
            {
               writeErrorCounter.increment();
            }
         }
      }
      if (status != TPCANStatus.PCAN_ERROR_OK)
      {
         writeErrorCounter.increment();
      }
   }
}