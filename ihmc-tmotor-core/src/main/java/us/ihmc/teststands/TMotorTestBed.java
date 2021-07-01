package us.ihmc.teststands;

import gnu.trove.map.hash.TIntObjectHashMap;
import peak.can.basic.*;
import us.ihmc.commons.Conversions;
import us.ihmc.realtime.*;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensors.LoadStarILoad.serial.SerialLoadcell;
import us.ihmc.tMotorCore.CANMessages.TMotorCANReplyMessage;
import us.ihmc.tMotorCore.TMotor;
import us.ihmc.tMotorCore.TMotorVersion;
import us.ihmc.trajectories.EvaWalkingJointTrajectories;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

import static peak.can.basic.TPCANStatus.PCAN_ERROR_QRCVEMPTY;

public class TMotorTestBed extends RealtimeThread
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer;

   private static final double DT = 0.001;
   private static final MonotonicTime period = new MonotonicTime(0, Conversions.secondsToNanoseconds(DT));

   private final static PriorityParameters controllerPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);

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
   private final TIntObjectHashMap<TMotor> motors = new TIntObjectHashMap<>();
   private int[] motorIDs;
   private static final int RIGHT_HIP_CAN_ID = 2;
   private static final int KNEE_CAN_ID = 9;

   // CAN-related goodies
   private PCANBasic can = new PCANBasic();
   private final TPCANHandle channel = TPCANHandle.PCAN_PCIBUS1;
   private final TPCANMsg receivedMsg = new TPCANMsg();
   private TPCANStatus status = null;

   // specialized YoVariables
   private final YoBoolean enableCANMsgs = new YoBoolean("enableCANMsgs", registry);
   private final YoBoolean resetCounters = new YoBoolean("resetCounters", registry);

   // debug
   private final YoInteger messagesInReadBus = new YoInteger("messagesInBus", registry);
   

   public TMotorTestBed(YoVariableServer yoVariableServer)
   {
      super(controllerPriority, new PeriodicParameters(period));

      CPUDMALatency.setLatency(0);

      this.yoVariableServer = yoVariableServer;
      yoVariableServer.setMainRegistry(registry, null);

      TMotor shoulderMotor = new TMotor(RobotSide.RIGHT, RIGHT_HIP_CAN_ID, TMotorVersion.AK109, DT, yoTime, registry);
      TMotor elbowMotor = new TMotor(RobotSide.RIGHT, KNEE_CAN_ID, TMotorVersion.AK109, DT, yoTime, registry);
      motors.put(shoulderMotor.getID(), shoulderMotor);
      motors.put(elbowMotor.getID(), elbowMotor);
      motorIDs = motors.keys();

      receivedMsg.setLength((byte) 6);
      enableCANMsgs.set(true);
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
         
         if(resetCounters.getBooleanValue()) 
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
            compute(yoTime.getDoubleValue());
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
      while(readStatus != PCAN_ERROR_QRCVEMPTY)
      {
         if (readStatus == TPCANStatus.PCAN_ERROR_OK)
         {
            int id = TMotorCANReplyMessage.getID(receivedMsg);
            if(motors.containsKey(id))
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

   private void compute(double time)
   {
      for(int id = 0; id < motorIDs.length; id++)
      {
         motors.get(motorIDs[id]).update(time);
      }
   }

   private void write()
   {
      for(int id = 0; id < motorIDs.length; id++)
      {
         TPCANMsg motorCommand = motors.get(motorIDs[id]).write();
         status = can.Write(channel, motorCommand);
         if (status != TPCANStatus.PCAN_ERROR_OK)
         {
            writeErrorCounter.increment();
         }
      }
   }

   public static void main(String[] args)
   {
      YoVariableServer yoVariableServer = new YoVariableServer(TMotorTestBed.class, null, new DataServerSettings(true), DT);
      TMotorTestBed testbed = new TMotorTestBed(yoVariableServer);

      testbed.initialize();

      yoVariableServer.start();

      testbed.start();
      testbed.join();

      yoVariableServer.close();

   }
}
