package us.ihmc.sensors.LoadStarILoad;

import javax.swing.JLabel;
import javax.swing.JToggleButton;
import javax.swing.event.ChangeEvent;

import jssc.SerialPort;
import jssc.SerialPortEventListener;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.value.ChangeListener;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizerStateListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.io.IOException;
import java.util.TooManyListenersException;

public class LoadStarMessageVisualizer implements SCSVisualizerStateListener, javax.swing.event.ChangeListener
{
   private static final String COMPort = "COM3";
   private LoadStarILoad load;
   private SerialPort serial;
   private ChangeListener<String> listener;
   //private final ArduinoSerialMessage arduinoSerialData;
   private final BooleanProperty connection = new SimpleBooleanProperty(false);

   private static final int DEFAULT_BUFFER_SIZE = 5000;
   private static final int DEFAULT_UPDATE_RATE = 1;

   // Data sent via CAN
   private final static String START_TRIANGLE_WAVE = "S";  // sawtooth feedforward command
   private final static String STOP_TRIANGLE_WAVE = "Z";   // zero feedforward command
   private final static String CHANGE_TRIANGLE_AMPLITUDE = "A";
   private final static String CHANGE_KT = "K";

   // Data received via CAN
   private YoDouble measuredForce;
   private JToggleButton tareButton;


   public LoadStarMessageVisualizer() throws IOException
   {
      SCSVisualizer scsVisualizer = new SCSVisualizer(DEFAULT_BUFFER_SIZE);
      scsVisualizer.setVariableUpdateRate(DEFAULT_UPDATE_RATE);
      scsVisualizer.addSCSVisualizerStateListener(this);
      YoVariableClient client = new YoVariableClient(scsVisualizer);

      client.startWithHostSelector();

      load = new LoadStarILoad(COMPort);
      //arduinoSerialData = new ArduinoSerialMessage();
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoRegistry registry) throws IOException
   {
      LogTools.info("Starting iLoad visualizer");
      scs.setFastSimulate(true);

      setupYoVariables(registry);

      startSerialConnection();

      // Show connection status
      scs.addJLabel(new JLabel("Connected: "+ connection.getValue()));

//      tareButton = new JToggleButton("Tare");
//      tareButton.addChangeListener(this);
//      scs.addButton(tareButton);

      // Add button to update PD gains
      //        JButton updateControlGains = new JButton("Update PD Gains");
      //        updateControlGains.addChangeListener(e->
      //                serial.write(updateControlGains.isSelected() ?
      //                        (float) desiredKp.getValue() : (float) controllerLowLevelKp.getValue()));
      //        updateControlGains.addChangeListener(e->controllerLowLevelKp.set(desiredKp.getDoubleValue()));//update
      //        scs.addButton(updateControlGains);
   }

   private void setupYoVariables(YoRegistry registry)
   {
      measuredForce = (YoDouble) registry.findVariable("measuredForce");
//      measuredPosition = (YoDouble) registry.findVariable("measuredPosition");
//      measuredVelocity = (YoDouble) registry.findVariable("measuredVelocity");
//      measuredTorque = (YoDouble) registry.findVariable("measuredTorque");
//      feedforwardTorque = (YoDouble) registry.findVariable("feedforwardTorque");
//      controllerLowLevelKp = (YoDouble) registry.findVariable("controllerLowLevelKp");
//      desiredKp = (YoDouble) registry.findVariable("desiredKp");
//      controllerLowLevelKd = (YoDouble) registry.findVariable("controllerLowLevelKd");
//      desiredMotorPosition = (YoDouble) registry.findVariable("desiredMotorPosition");
//      desiredMotorVelocity = (YoDouble) registry.findVariable("desiredMotorVelocity");
//      estimateDesiredMotorTorque = (YoBoolean) registry.findVariable("estimateDesiredMotorTorque");
//      motorID = (YoInteger) registry.findVariable("motorID");
//
//      currentTriangleWaveAmplitude = (YoDouble) registry.findVariable("currentTriangleWaveAmplitude");
//      desiredTriangleWaveAmplitude = (YoDouble) registry.findVariable("desiredTriangleWaveAmplitude");
//      currentKtCorrectingFactor = (YoDouble) registry.findVariable("currentKtCorrectingFactor");
//      desiredKtCorrectingFactor = (YoDouble) registry.findVariable("desiredKtCorrectingFactor");

//       setup YoVariable change listeners
//              desiredKp.addListener(e->serial.write((float) desiredKp.getValue()));// update CAN
//              desiredKp.addListener(e->
//                      controllerLowLevelKp.set(desiredKp.getDoubleValue()));   // update YoVar
//
//      desiredTriangleWaveAmplitude.addListener(e->
//                                                     currentTriangleWaveAmplitude.set(desiredTriangleWaveAmplitude.getDoubleValue()));   // update YoVar
//      desiredTriangleWaveAmplitude.addListener(e->serial.write(CHANGE_TRIANGLE_AMPLITUDE));
//      desiredTriangleWaveAmplitude.addListener(e->
//                                                     serial.write((float) desiredTriangleWaveAmplitude.getValue()));// update CAN
//
//      desiredKtCorrectingFactor.addListener(e->
//                                                  currentKtCorrectingFactor.set(desiredKtCorrectingFactor.getDoubleValue()));   // update YoVar
//      desiredKtCorrectingFactor.addListener(e->serial.write(CHANGE_KT));
//      desiredKtCorrectingFactor.addListener(e->
//                                                  serial.write((float) desiredKtCorrectingFactor.getValue()));// update CAN

//        controllerLowLevelKd.addListener(e->serial.write((float) controllerLowLevelKd.getValue()));
   }

   private void startSerialConnection()
   {

      listener = (ov, t, msg) -> {
         try {
            load.outputWeightOnce();
//            System.out.println("Info: " + msg);
            updateYoVariables(load.getForceNewton());
         } catch (NumberFormatException nfe) {
            System.out.println("NFE: " + msg + " " + nfe.toString());
         }
      };

      //serial.addEventListener((SerialPortEventListener) listener);

      //serial.connect();
      //connection.set(!serial.getPortName().isEmpty());
   }

   private void updateYoVariables(double doub) {
      measuredForce.set(doub);
   }

//   private void updateYoVariables(ArduinoSerialMessage message)
//   {
//      measuredPosition.set(message.getPositionInput());
//      measuredVelocity.set(message.getVelocityInput());
//      measuredTorque.set(message.getMeasuredTorque());
//      feedforwardTorque.set(message.getTorqueFfwdInput());
//      controllerLowLevelKp.set(message.getControllerKp());
//      controllerLowLevelKd.set(message.getControllerKd());
//      desiredMotorPosition.set(message.getDesiredMotorPosition());
//      desiredMotorVelocity.set(message.getDesiredMotorVelocity());
//      motorID.set(message.getMotorID());
//   }

   public static void main(String[] args) throws IOException
   {
      new LoadStarMessageVisualizer();
   }

   @Override
   public void stateChanged(ChangeEvent e)
   {
      //TODO: Implement this
   }

// @Override
// public void stateChanged(ChangeEvent e)
// {
//    serial.write(tareButton.isSelected() ? START_TRIANGLE_WAVE : STOP_TRIANGLE_WAVE);
//    estimateDesiredMotorTorque.set(tareButton.isSelected());
// }


}