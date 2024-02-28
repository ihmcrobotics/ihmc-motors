package us.ihmc.sensors.loadStarILoad;

import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import jssc.SerialPort;
import jssc.SerialPortException;
import us.ihmc.sensors.loadStarILoad.serial.SerialPortReader;
import us.ihmc.sensors.loadStarILoad.serial.SerialPortTools;
import us.ihmc.sensors.loadStarILoad.settings.LoadStarILoadCommandEnum;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.io.IOException;

/**
 * See http://www.loadstarsensors.com/hyperterminal.html
 */
public class LoadStarILoad
{
   private static final int TIMEOUT = 10000;
   private static final int BAUDRATE = 9600;    // probably doesn't matter, as mentioned in the documentation
   private SerialPort serialPort;

   private LoadStarILoadWriter loadStarILoadWriter;
   private SerialPortReader serialPortReader;
   private LoadStarILoadParser parser;
   private LoadStarILoadCallback loadStarILoadCallback;

   private Double force;
   private StringProperty forceString = new SimpleStringProperty("");

   public LoadStarILoad(String portName) throws IOException, SerialPortException
   {
      SerialPortTools.printSerialPortNames();
      connect(portName);
      addShutdownHook();
   }

   public LoadStarILoad(String portName, YoRegistry registry) throws IOException, SerialPortException
   {
      SerialPortTools.printSerialPortNames();
      connect(portName);
      addShutdownHook();
   }

   public void connect(String portName) throws IOException, SerialPortException
   {
      System.out.println("Attempting to open serial port: \"" + portName + "\"");

      serialPort = SerialPortTools.openSerialPort(portName,
                                                  this.getClass().getName(),
                                                  BAUDRATE,
                                                  SerialPort.DATABITS_8,
                                                  SerialPort.STOPBITS_1,
                                                  SerialPort.PARITY_NONE,
                                                  SerialPort.FLOWCONTROL_NONE,
                                                  TIMEOUT);

      //      serialPort.setEventsMask('A');
      //      serialPort.addEventListener(e ->
      //      {
      //         if (e.isRXCHAR())
      //         {
      //            int[] byteArray = new int[0];
      //            try {
      //               byteArray = serialPort.readIntArray();
      //            } catch (SerialPortException ex) {
      //               ex.printStackTrace();
      //            }
      //            if (byteArray != null) {
      //               for (int byteValue : byteArray)
      //                  parser.parseByte(byteValue);
      //               force = parser.getForce();
      //               forceString.set(force.toString());
      //               System.out.println(forceString);
      //            }
      //         }
      //      });

      //      InputStream inputStream = serialPort.getInputStream();
      //      OutputStream outputStream = serialPort.getOutputStream();

      loadStarILoadWriter = new LoadStarILoadWriter(serialPort);

      loadStarILoadCallback = new LoadStarILoadCallback();
      parser = new LoadStarILoadParser(loadStarILoadCallback);
      //PriorityParameters priorityParameters = new PriorityParameters(PriorityParameters.getMaximumPriority()); //cant find these
      serialPortReader = new SerialPortReader(serialPort, parser);
      SerialPortTools.createReaderThread(serialPortReader); //changed from: (serialPortReader, priorityParameters)
   }

   public void disconnect() throws IOException, SerialPortException
   {
      //      loadStarILoadWriter.disconnect();
      //      serialPortReader.disconnect();
      //      SerialPortTools.closeSerialPort(serialPort);
      serialPort.closePort();
   }

   public void ping()
   {
      doCommand(LoadStarILoadCommandEnum.PING);
   }

   public void tare()
   {
      doCommand(LoadStarILoadCommandEnum.TARE);
   }

   public void outputWeightOnce()
   {
      doCommand(LoadStarILoadCommandEnum.OUTPUT_WEIGHT_ONCE);
   }

   public void outputWeightContinuously()
   {
      doCommand(LoadStarILoadCommandEnum.OUTPUT_WEIGHT_CONTINUOUSLY);
   }

   public void outputTemperature()
   {
      doCommand(LoadStarILoadCommandEnum.OUTPUT_TEMPERATURE);
   }

   public void outputLoadCapacity()
   {
      doCommand(LoadStarILoadCommandEnum.OUTPUT_LOAD_CAPACITY);
   }

   public void outputSerialNumber()
   {
      doCommand(LoadStarILoadCommandEnum.OUTPUT_SERIAL_NUMBER);
   }

   public void outputModelNumber()
   {
      doCommand(LoadStarILoadCommandEnum.OUTPUT_MODEL_NUMBER);
   }

   public double getForceNewton()
   {
      return loadStarILoadCallback.getForceNewton();
   }

   public double getForcePound()
   {
      return loadStarILoadCallback.getForcePound();
   }

   public SerialPort getSerialPort()
   {
      return serialPort;
   }

   private void doCommand(LoadStarILoadCommandEnum command)
   {
      parser.expectCommand(null);
      loadStarILoadWriter.sendCommand(LoadStarILoadCommandEnum.PING); // stop continuous stuff
      sleep(100L); // wait until done receiving data
      parser.expectCommand(command);
      loadStarILoadWriter.sendCommand(command);
      sleep(100L); // don't allow any new commands until the last one is processed.
   }

   private void sleep(long millis)
   {
      try
      {
         Thread.sleep(millis);
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   private void addShutdownHook()
   {
      Runnable shutdownHookRunnable = new Runnable()
      {
         public void run()
         {
            System.out.println("LoadStarILoad: Disconnecting.");

            try
            {
               disconnect();
            }
            catch (IOException | SerialPortException e)
            {
               // do nothing
            }
         }
      };
      Thread hook = new Thread(shutdownHookRunnable);
      Runtime.getRuntime().addShutdownHook(hook);
   }

   public StringProperty getLine()
   {
      return forceString;
   }

   public SerialPortReader getSerialPortReader()
   {
      return serialPortReader;
   }
}
