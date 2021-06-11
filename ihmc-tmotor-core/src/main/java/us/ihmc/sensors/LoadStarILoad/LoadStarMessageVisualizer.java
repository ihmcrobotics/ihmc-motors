package us.ihmc.sensors.LoadStarILoad;

import javax.swing.*;
import javax.swing.event.ChangeEvent;

import javafx.beans.value.ChangeListener;
import jssc.SerialPort;
import jssc.SerialPortEventListener;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import jssc.SerialPortException;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizerStateListener;
import us.ihmc.sensors.LoadStarILoad.serial.Serial;
import us.ihmc.sensors.LoadStarILoad.testbed.LoadStarTestBed;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.io.IOException;

public class LoadStarMessageVisualizer implements SCSVisualizerStateListener, javax.swing.event.ChangeListener
{
   private static final String COMPort = "COM9";
   private static final double DT = 0.001;
   private LoadStarILoad load;
   private SerialPort serialPort;
   private Serial serial;
   private ChangeListener<String> listener;   //private final ArduinoSerialMessage arduinoSerialData;
   private final BooleanProperty connection = new SimpleBooleanProperty(false);

   private static final int DEFAULT_BUFFER_SIZE = 5000;
   private static final int DEFAULT_UPDATE_RATE = 1;

   // Data sent via CAN
   private final static String START_TRIANGLE_WAVE = "S";  // sawtooth feedforward command
   private final static String STOP_TRIANGLE_WAVE = "Z";   // zero feedforward command
   private final static String CHANGE_TRIANGLE_AMPLITUDE = "A";
   private final static String CHANGE_KT = "K";

   // Data received via CAN
   private YoDouble measuredForceN;
   private YoDouble measuredForceLb;
   private YoBoolean readingOn;
   private JToggleButton tareButton;

   private YoRegistry registry;
   private LoadStarTestBed testbed;
   private YoVariableServer yoVariableServer;
   private Thread thread;


   public LoadStarMessageVisualizer() throws IOException
   {
      SCSVisualizer scsVisualizer = new SCSVisualizer(DEFAULT_BUFFER_SIZE);
      scsVisualizer.setVariableUpdateRate(DEFAULT_UPDATE_RATE);
      scsVisualizer.addSCSVisualizerStateListener(this);

      YoVariableClient client = new YoVariableClient(scsVisualizer);
      client.startWithHostSelector();
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoRegistry registry) throws InterruptedException, IOException {
      LogTools.info("Starting iLoad visualizer");
      scs.setFastSimulate(true);

      this.registry = registry;
//      load = new LoadStarILoad(COMPort);

      try {
         startSerialConnection();
      } catch (Exception e) {
         LogTools.info(e.getMessage());
      }

      setupYoVariables(registry);

      // Show connection status
      LogTools.info("Connected: " + connection.getValue());
      scs.addJLabel(new JLabel("Connected: "+ connection.getValue()));

      tareButton = new JToggleButton("Tare");
      tareButton.addChangeListener(this);
      scs.addButton(tareButton);

//       Add button to update PD gains
//              JButton updateControlGains = new JButton("Update PD Gains");
//              updateControlGains.addChangeListener(e->
//                      serial.write(updateControlGains.isSelected() ?
//                              (float) desiredKp.getValue() : (float) controllerLowLevelKp.getValue()));
//              updateControlGains.addChangeListener(e->controllerLowLevelKp.set(desiredKp.getDoubleValue()));//update
//              scs.addButton(updateControlGains);
   }

   private void setupYoVariables(YoRegistry registry)
   {
      measuredForceN = (YoDouble) registry.findVariable("measuredForceN");
      measuredForceLb = (YoDouble) registry.findVariable("measuredForceLb");
      readingOn = (YoBoolean) registry.findVariable("readingOn");
   }

   private void startSerialConnection() throws SerialPortException {
      listener = (ov, t, msg) -> {
         try {
            System.out.println("Listening");
            load.outputWeightOnce();
            updateYoVariables();
         } catch (NumberFormatException nfe) {
            System.out.println("NFE: " + msg + " " + nfe.toString());
         }
      };

      serial = new Serial(COMPort);
      serialPort = load.getSerialPort();
      serial.connect(serialPort);

      serial.getLine().addListener(listener);
      connection.set(!serial.getPortName().isEmpty());
   }

   private void updateYoVariables() {
      load.outputWeightOnce();
      measuredForceN.set(load.getForceNewton());
      measuredForceLb.set(load.getForcePound());
   }

   public static void main(String[] args) throws IOException {
      new LoadStarMessageVisualizer();
   }

 @Override
 public void stateChanged(ChangeEvent e)
 {
    System.out.println("Event changed");
    //serial.write(tareButton.isSelected() ? START_TRIANGLE_WAVE : STOP_TRIANGLE_WAVE);
    readingOn.set(tareButton.isSelected());
 }


}