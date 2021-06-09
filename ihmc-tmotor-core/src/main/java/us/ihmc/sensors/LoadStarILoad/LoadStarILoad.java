package us.ihmc.sensors.LoadStarILoad;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import jssc.SerialPort;
import jssc.SerialPortException;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.sensors.LoadStarILoad.serial.SerialPortReader;
import us.ihmc.sensors.LoadStarILoad.serial.SerialPortTools;
import us.ihmc.sensors.LoadStarILoad.settings.LoadStarILoadCommandEnum;

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

   public LoadStarILoad(String portName) throws IOException
   {
      SerialPortTools.printSerialPortNames();
      connect(portName);
      addShutdownHook();
   }

   public void connect(String portName) throws IOException
   {
      System.out.println("Attempting to open serial port: \"" + portName + "\"");

      serialPort = SerialPortTools.openSerialPort(portName, this.getClass().getName(), BAUDRATE, SerialPort.DATABITS_8, SerialPort.STOPBITS_1,
              SerialPort.PARITY_NONE, SerialPort.FLOWCONTROL_NONE, TIMEOUT);

//      InputStream inputStream = serialPort.getInputStream();
//      OutputStream outputStream = serialPort.getOutputStream();

      loadStarILoadWriter = new LoadStarILoadWriter(serialPort);

      loadStarILoadCallback = new LoadStarILoadCallback();
      parser = new LoadStarILoadParser(loadStarILoadCallback);
      //PriorityParameters priorityParameters = new PriorityParameters(PriorityParameters.getMaximumPriority()); //cant find these
      serialPortReader = new SerialPortReader(serialPort, parser);
      SerialPortTools.createReaderThread(serialPortReader); //changed from: (serialPortReader, priorityParameters)
   }

   public void disconnect() throws IOException, SerialPortException {
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

   public double getForcePound() {
      return loadStarILoadCallback.getForcePound();
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
      } catch (InterruptedException e)
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
}
