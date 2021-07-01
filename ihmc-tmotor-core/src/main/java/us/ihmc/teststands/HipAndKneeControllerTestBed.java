package us.ihmc.teststands;

import gnu.trove.map.hash.TIntObjectHashMap;
import peak.can.basic.*;
import us.ihmc.commons.Conversions;
import us.ihmc.eva.controlModules.constraints.EvaGearedJointConstraint;
import us.ihmc.eva.model.EvaExoskeletonModel;
import us.ihmc.eva.model.EvaExoskeletonVersion;
import us.ihmc.eva.model.EvaModelFactory;
import us.ihmc.eva.model.robotVersions.RightHipAndKneeTestStandJointMap;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.realtime.*;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tMotorCore.CANMessages.TMotorCANReplyMessage;
import us.ihmc.tMotorCore.TMotor;
import us.ihmc.tMotorCore.parameters.EvaMotorIDParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static peak.can.basic.TPCANStatus.PCAN_ERROR_QRCVEMPTY;

/**
 */
public class HipAndKneeControllerTestBed extends RealtimeThread
{
   private static final boolean USE_DC = false;
   private static final double DT = 0.001;
   private static final MonotonicTime period = new MonotonicTime(0, Conversions.secondsToNanoseconds(DT));
   private final static PriorityParameters controllerPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer;

   private final YoLong tickStartTimeInNanos = new YoLong("tickStartTimeInNanos", registry);
   private final YoLong doControlTime = new YoLong("doControlTime", registry);
   private final YoLong computeTime = new YoLong("computeTime", registry);
   private final YoLong canReadTime = new YoLong("canReadTime", registry);
   private final YoLong canWriteTime = new YoLong("canWriteTime", registry);
   private final YoLong effectiveDT = new YoLong("effectiveDT", registry);

   private final YoLong controllerTimeInNanos = new YoLong("controllerTimeInNanos", registry);
   private final YoDouble controllerTimeInSeconds = new YoDouble("controllerTimeInSeconds", registry);

   private final YoLong readErrorCounter = new YoLong("readErrorCounter", registry);
   private final YoLong writeErrorCounter = new YoLong("writeErrorCounter", registry);

   private static final EvaExoskeletonVersion evaVersion = EvaExoskeletonVersion.SIMPLIFIED_FIXED_HIPS;
   private HashMap<TMotor, OneDoFJointBasics> motorToJointMap = new HashMap<>();
   private FullRobotModel fullRobotModel;

   // motors in CAN bus
   private final TIntObjectHashMap<TMotor> motors = new TIntObjectHashMap<>();
   private int[] motorIDs;

   // CAN-related goodies
   private PCANBasic can = new PCANBasic();
   private final TPCANHandle channel = TPCANHandle.PCAN_PCIBUS1;
   private final TPCANMsg receivedMsg = new TPCANMsg();
   private TPCANStatus status = null;

   // debug YoVariables
   private final YoBoolean enableCANMsgs = new YoBoolean("enableCANMsgs", registry);
   private final YoBoolean resetCounters = new YoBoolean("resetCounters", registry);


   public HipAndKneeControllerTestBed(EvaModelFactory modelFactory, YoVariableServer yoVariableServer)
   {
      super(controllerPriority, new PeriodicParameters(period));
      LogTools.info("here " + System.getProperty("user.dir"));
      CPUDMALatency.setLatency(0);

      this.yoVariableServer = yoVariableServer;
      fullRobotModel = modelFactory.createFullRobotModel();

      yoVariableServer.setMainRegistry(registry, HipAndKneeControllerTestBed.createYoVariableServerJointList(fullRobotModel.getElevator()), null);

      EvaMotorIDParameters evaMotorParameters = new EvaMotorIDParameters(evaVersion);
      for(RightHipAndKneeTestStandJointMap.Joints joint : RightHipAndKneeTestStandJointMap.Joints.values())
      {
         if(evaMotorParameters.isMotorPresent(joint))
         {
            int id = evaMotorParameters.getJointMotorID(joint);
            int zAxisDirection = evaMotorParameters.getZAxisSign(joint);
            TMotor motor = new TMotor(RobotSide.RIGHT, id, zAxisDirection, DT, controllerTimeInSeconds, registry);
            motors.put(id, motor);
            motorToJointMap.put(motor, fullRobotModel.getOneDoFJointByName(joint.getName()));
         }
      }
      motorIDs = motors.keys();

      receivedMsg.setLength((byte) 6);
      enableCANMsgs.set(true);
   }

   public static List<JointBasics> createYoVariableServerJointList(RigidBodyBasics rootBody)
   {
      List<JointBasics> joints = new ArrayList<>();

      for (JointBasics joint : rootBody.childrenSubtreeIterable())
      {
         joints.add(joint);
      }

      return joints;
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

   private void motorRead()
   {
      TPCANStatus readStatus = can.Read(channel, receivedMsg, null);

      while(readStatus != PCAN_ERROR_QRCVEMPTY)
      {
         if (readStatus == TPCANStatus.PCAN_ERROR_OK)
         {
            int id = TMotorCANReplyMessage.getID(receivedMsg);
            if(motors.containsKey(id))
               motors.get(id).read(receivedMsg);
         }
         else
         {
            readErrorCounter.increment();
         }
         readStatus = can.Read(channel, receivedMsg, null);
      }

      // update robot OneDofJointData
      for(int index = 0; index < motorIDs.length; index++)
      {
         TMotor motor = motors.get(motorIDs[index]);
         OneDoFJointBasics joint = motorToJointMap.get(motor);
         updateJointState(joint, motor);
      }
   }

   private void updateJointState(OneDoFJointBasics joint, TMotor motor)
   {
      joint.setQ(motor.getPosition());
      joint.setQd(motor.getVelocity());
      joint.setTau(motor.getTorque());
   }

   private void motorCompute(double time)
   {
      for(int id = 0; id < motorIDs.length; id++)
      {
         motors.get(motorIDs[id]).update(time);
      }
   }

   private void motorWrite()
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

   /**
    * Callback called cyclically to do the control loop. Will not get called till all actuators are
    * online. Will also not get called when a deadline is missed or a datagram got lost.
    */
   @Override
   public void run()
   {

      long controllerStartTime = System.nanoTime();
      while (true)
      {
         effectiveDT.set(System.nanoTime() - tickStartTimeInNanos.getLongValue());
         tickStartTimeInNanos.set(System.nanoTime());

         controllerTimeInNanos.set(System.nanoTime() - controllerStartTime);
         controllerTimeInSeconds.set(Conversions.nanosecondsToSeconds(tickStartTimeInNanos.getLongValue() - controllerStartTime));

         if(resetCounters.getBooleanValue())
         {
            readErrorCounter.set(0);
            writeErrorCounter.set(0);
            resetCounters.set(false);
         }

         if (enableCANMsgs.getBooleanValue())
         {

            long canReadStartTime = System.nanoTime();
            motorRead();
            canReadTime.set(System.nanoTime() - canReadStartTime);

            long computeStartTime = System.nanoTime();
            motorCompute(controllerTimeInSeconds.getDoubleValue());
            computeTime.set(System.nanoTime() - computeStartTime);

            long canWriteStartTime = System.nanoTime();
            motorWrite();
            canWriteTime.set(System.nanoTime() - canWriteStartTime);

            // TODO safe sentinel
         }
         // wait to free sent queue
         doReporting();
         doControlTime.set(System.nanoTime() - tickStartTimeInNanos.getLongValue());
         waitForNextPeriod();
      }

   }

   /**
    * Callback called cyclically to do reporting and logging. Will always get called, even when the
    * controller is not yet online or a deadline is missed. Make sure this function returns quickly.
    */
   protected void doReporting()
   {

      //skip here if needed
      yoVariableServer.update(controllerTimeInNanos.getLongValue());
   }


   public static void main(String[] args)
   {

      EvaModelFactory modelFactory = new EvaModelFactory(evaVersion, null);
      LogModelProvider logModelProvider = modelFactory.createLogModelProvider();

      YoVariableServer yoVariableServer = new YoVariableServer(HipAndKneeControllerTestBed.class, logModelProvider, new DataServerSettings(true), DT);

      HipAndKneeControllerTestBed testbed = new HipAndKneeControllerTestBed(modelFactory, yoVariableServer);
      testbed.initialize();

      yoVariableServer.start();
      testbed.start();

      testbed.join();
      yoVariableServer.close();
   }
}
