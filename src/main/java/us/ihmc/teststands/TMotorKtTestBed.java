package us.ihmc.teststands;

import peak.can.basic.PCANBasic;
import peak.can.basic.TPCANBaudrate;
import peak.can.basic.TPCANHandle;
import peak.can.basic.TPCANMsg;
import peak.can.basic.TPCANStatus;
import peak.can.basic.TPCANType;
import us.ihmc.can.CANTools;
import us.ihmc.commons.Conversions;
import us.ihmc.etherCAT.master.EtherCATRealtimeThread;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.slaves.beckhoff.EK1100;
import us.ihmc.etherCAT.slaves.beckhoff.EL3062;
import us.ihmc.etherCAT.slaves.beckhoff.EL3104;
import us.ihmc.etherCAT.slaves.beckhoff.EL9510;
import us.ihmc.etherCAT.slaves.beckhoff.YoAnalogSignalWrapper;
import us.ihmc.etherCAT.slaves.beckhoff.YoEL3062;
import us.ihmc.etherCAT.slaves.beckhoff.YoEL3104;
import us.ihmc.log.LogTools;
import us.ihmc.process.LinuxProcess;
import us.ihmc.process.Scheduler;
import us.ihmc.process.SchedulerAlgorithm;
import us.ihmc.realtime.CPUDMALatency;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.ButterworthFilteredYoVariable;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorNew;
import us.ihmc.tMotorCore.TMotor;
import us.ihmc.tMotorCore.TMotorLowLevelController;
import us.ihmc.tMotorCore.TMotorVersion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

import java.io.IOException;
import java.util.List;

import static peak.can.basic.TPCANStatus.PCAN_ERROR_QRCVEMPTY;

/**
 *
 */
public class TMotorKtTestBed extends EtherCATRealtimeThread
{
   private static final boolean USE_DC = false;
   private static final double DT = 0.001;
   private final static PriorityParameters controllerPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);
   private static final int CAN_ID = 11; // 10; // 1; //  2; // 9; //
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
   private final YoEL3104 yoEL3104;
   private final YoEL3062 yoEL3062;
   private final TMotor tMotor;
   private final TMotorLowLevelController motorController;
   private final YoFunctionGeneratorNew functionGenerator;
   private final YoEnum<Slave.State> ek1100State = new YoEnum<>("ek1100State", registry, Slave.State.class);
   private final YoEnum<Slave.State> el3104State = new YoEnum<>("el3104State", registry, Slave.State.class);
   private final TPCANHandle channel1 = TPCANHandle.PCAN_PCIBUS1;
   //   private final TPCANHandle channel2 = TPCANHandle.PCAN_PCIBUS2;
   private final TPCANMsg receivedMsg = new TPCANMsg();
   private final YoAnalogSignalWrapper torqueSensorProcessor;
   private final ButterworthFilteredYoVariable filteredTorque;
   private final YoDouble alphaLoadcell = new YoDouble("alphaLoadcell", registry);
   // CAN-related goodies
   private PCANBasic can = new PCANBasic();
   private TPCANStatus status = null;
   private YoAnalogSignalWrapper tempSensorProcessor;

   public TMotorKtTestBed(String iface, MonotonicTime period, boolean enableDC, YoVariableServer yoVariableServer)
   {
      super(iface, controllerPriority, period, enableDC, 250000);
      LogTools.info("here " + System.getProperty("user.dir"));
      setEtherCATPriorityForInterface(iface);
      CPUDMALatency.setLatency(0);

      this.yoVariableServer = yoVariableServer;
      yoVariableServer.setMainRegistry(registry, null);

      EK1100 ek1100 = new EK1100(0, 0);

      EL3104 analogInput = new EL3104(0, 1);
      yoEL3104 = new YoEL3104(analogInput, registry);

      EL3062 el3062 = new EL3062(0, 2);
      yoEL3062 = new YoEL3062(el3062, registry);

      EL9510 el9510 = new EL9510(0, 3);

      tMotor = new TMotor(CAN_ID, "tMotor", TMotorVersion.AK809, DT, registry);
      motorController = new TMotorLowLevelController("tMotorController", tMotor, registry);
      motorController.setUnsafeOutputSpeed(12.0);

      functionGenerator = new YoFunctionGeneratorNew("functionGenerator", DT, registry);

      receivedMsg.setLength((byte) 6);

      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(140, DT);
      alphaLoadcell.set(alpha);

      ek1100State.set(ek1100.getState());
      el3104State.set(analogInput.getState());

      torqueSensorProcessor = new YoAnalogSignalWrapper("torqueCell", yoEL3104, 0, registry);
      torqueSensorProcessor.setCoeffs(2.0, 200.0 / 5.0, 0.0); // .setCoeffs(-7.0, 1140.0, 0.0); //
      registerSlave(ek1100);
      registerSlave(analogInput);
      registerSlave(el3062);
      registerSlave(el9510);

      tempSensorProcessor = new YoAnalogSignalWrapper("motorTemp", yoEL3062, 0, registry);
      tempSensorProcessor.setCoeffs(5.5537, 1.6913, 1.1642);

      setRequireAllSlaves(false);

      filteredTorque = new ButterworthFilteredYoVariable("filteredTorque",
                                                         registry,
                                                         alphaLoadcell,
                                                         torqueSensorProcessor.getResultYoVariable(),
                                                         ButterworthFilteredYoVariable.ButterworthFilterType.LOW_PASS);
   }

   public static void main(String[] args)
   {
      String iface = args[0];
      MonotonicTime period = new MonotonicTime(0, Conversions.secondsToNanoseconds(DT));

      YoVariableServer yoVariableServer = new YoVariableServer(TMotorKtTestBed.class, null, new DataServerSettings(true), period.asSeconds());

      TMotorKtTestBed testbed = new TMotorKtTestBed(iface, period, USE_DC, yoVariableServer);
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

      //      status = can.Initialize(channel1, TPCANBaudrate.PCAN_BAUD_1M, TPCANType.PCAN_TYPE_NONE, 0, (short) 0);
      //      LogTools.info("channel 1 " + channel1.getValue() + " initialized. Status: " + status);

      status = can.Initialize(channel1, TPCANBaudrate.PCAN_BAUD_1M, TPCANType.PCAN_TYPE_NONE, 0, (short) 0);
      LogTools.info("channel 1 " + channel1.getValue() + " initialized. Status: " + status);

      //     can.SetRcvEvent(channel);
   }

   private void motorRead()
   {
      TPCANStatus readStatus;
      while ((readStatus = can.Read(channel1, receivedMsg, null)) != PCAN_ERROR_QRCVEMPTY)
      {
         if (readStatus == TPCANStatus.PCAN_ERROR_OK)
         {
            int id = CANTools.getID(receivedMsg);
            if (id == CAN_ID)
               motorController.read(receivedMsg);
         }
         else
         {
            readErrorCounter.increment();
         }
      }
   }

   private void motorWrite()
   {
      TPCANMsg motorCommand = tMotor.getCommandedMsg();
      status = can.Write(channel1, motorCommand);
      if (status != TPCANStatus.PCAN_ERROR_OK)
      {
         writeErrorCounter.increment();
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

   private void setDesireds()
   {
      functionGenerator.update();
      double tau_d = functionGenerator.getValue();

      motorController.setDesiredPosition(0.0);
      motorController.setDesiredVelocity(0.0);
      motorController.setDesiredTorque(tau_d);
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
      yoEL3062.read();

      motorRead();
      filteredTorque.update();

      torqueSensorProcessor.update();
      tempSensorProcessor.update();
      setDesireds();
      motorController.doControl();

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
}
