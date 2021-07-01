package us.ihmc.teststands;

import gnu.trove.map.hash.TIntObjectHashMap;
import peak.can.basic.*;
import us.ihmc.commons.Conversions;
import us.ihmc.etherCAT.master.EtherCATRealtimeThread;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.slaves.beckhoff.EK1100;
import us.ihmc.etherCAT.slaves.beckhoff.EL3104;
import us.ihmc.etherCAT.slaves.beckhoff.YoAnalogSignalWrapper;
import us.ihmc.etherCAT.slaves.beckhoff.YoEL3104;
import us.ihmc.eva.model.EvaExoskeletonModel;
import us.ihmc.eva.model.EvaExoskeletonVersion;
import us.ihmc.eva.model.robotVersions.RightHipAndKneeTestStandJointMap;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.process.LinuxProcess;
import us.ihmc.process.Scheduler;
import us.ihmc.process.SchedulerAlgorithm;
import us.ihmc.realtime.*;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.math.filters.ButterworthFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tMotorCore.CANMessages.TMotorCANReplyMessage;
import us.ihmc.tMotorCore.TMotor;
import us.ihmc.tMotorCore.parameters.EvaMotorIDParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static peak.can.basic.TPCANStatus.PCAN_ERROR_QRCVEMPTY;

/**
 */
public class HipAndKneeForceControllerTestBed extends EtherCATRealtimeThread
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

   private final EK1100 ek1100;
   private final EL3104 analogInput;
   private final YoEL3104 yoEL3104;
   private final YoEnum<Slave.State> ek1100State = new YoEnum<>("ek1100State", registry, Slave.State.class);
   private final YoEnum<Slave.State> el3104State = new YoEnum<>("el3104State", registry, Slave.State.class);
   private final YoAnalogSignalWrapper torqueSensorProcessor;
   private final ButterworthFilteredYoVariable filteredTorque;
   private final YoDouble alphaLoadcell = new YoDouble("alphaLoadcell", registry);

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


   public HipAndKneeForceControllerTestBed(String iface, MonotonicTime period, boolean enableDC, YoVariableServer yoVariableServer)
   {
      super(iface, controllerPriority, period, enableDC, 250000);
      LogTools.info("here " + System.getProperty("user.dir"));
      setEtherCATPriorityForInterface(iface);
      CPUDMALatency.setLatency(0);

      this.yoVariableServer = yoVariableServer;
      EvaExoskeletonModel robotModel = new EvaExoskeletonModel(evaVersion, EvaExoskeletonModel.RobotTarget.REAL_ROBOT);
      fullRobotModel = robotModel.createFullRobotModel();

      yoVariableServer.setMainRegistry(registry, HipAndKneeForceControllerTestBed.createYoVariableServerJointList(fullRobotModel.getElevator()), null);

      ek1100 = new EK1100(0, 0);
      analogInput = new EL3104(0, 1);
      yoEL3104 = new YoEL3104(analogInput, registry);

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
      alphaLoadcell.set(0.99);

      ek1100State.set(ek1100.getState());
      el3104State.set(analogInput.getState());

      torqueSensorProcessor = new YoAnalogSignalWrapper("torqueCell", yoEL3104, 0, registry);
      torqueSensorProcessor.setCoeffs(-7.0, 1140.0, 0.0); // .setCoeffs(2.0, 200.0 / 5.0, 0.0);
      registerSlave(ek1100);
      registerSlave(analogInput);

      filteredTorque = new ButterworthFilteredYoVariable("filteredTorque",
              registry, alphaLoadcell, torqueSensorProcessor.getResultYoVariable(), ButterworthFilteredYoVariable.ButterworthFilterType.LOW_PASS);
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

         joint.setQ(motor.getPosition());
         joint.setQd(motor.getVelocity());
         joint.setTau(motor.getTorque());
      }
   }

   private void readLoadCells()
   {
      filteredTorque.update();

      torqueSensorProcessor.update();
      for(int motor = 0; motor < motorIDs.length; motor++)
      {
         motors.get(motor).setMeasuredForce(filteredTorque.getDoubleValue());
      }

   }

   private void motorCompute()
   {
      for(int id = 0; id < motorIDs.length; id++)
      {
         motors.get(motorIDs[id]).update(controllerTimeInSeconds.getDoubleValue());
      }
   }

   private void motorWrite()
   {
      for(int motor = 0; motor < motorIDs.length; motor++)
      {
         TPCANMsg motorCommand = motors.get(motor).write();
         status = can.Write(channel, motorCommand);
         if (status != TPCANStatus.PCAN_ERROR_OK)
         {
            writeErrorCounter.increment();
         }
      }
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

      yoEL3104.read();
      motorRead();
      readLoadCells();
      motorCompute();

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

   public static void main(String[] args)
   {
      String iface = args[0];
      MonotonicTime period = new MonotonicTime(0, Conversions.secondsToNanoseconds(DT));

      YoVariableServer yoVariableServer = new YoVariableServer(HipAndKneeForceControllerTestBed.class, null, new DataServerSettings(true), period.asSeconds());

      HipAndKneeForceControllerTestBed testbed = new HipAndKneeForceControllerTestBed(iface, period, USE_DC, yoVariableServer);
      testbed.initialize();

      yoVariableServer.start();
      testbed.start();

      testbed.join();
      yoVariableServer.close();
   }
}