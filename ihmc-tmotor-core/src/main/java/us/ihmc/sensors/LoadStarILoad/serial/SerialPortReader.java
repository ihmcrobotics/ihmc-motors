package us.ihmc.sensors.LoadStarILoad.serial;

import jssc.SerialPort;
import jssc.SerialPortException;

import java.io.IOException;

public class SerialPortReader implements Runnable
{
   private final SerialPort serialPort;
   private final ByteParser parser;
   private boolean isConnected = true;

   public SerialPortReader(SerialPort serialPort, ByteParser parser) {
      this.serialPort = serialPort;
      this.parser = parser;
   }

   public void run()
   {
      try{
         System.out.println("Starting Serial port reader thread");
         
         while (isConnected)
         {
            int [] byteArray = serialPort.readIntArray();
            if (byteArray != null) {
               for(int byteValue : byteArray)
                  parser.parseByte(byteValue);
            }
         }
      }
      catch (SerialPortException e)
      {
         e.printStackTrace();
         throw new RuntimeException("Cannot read serial port");
      }
   }

   public void disconnect() throws IOException, SerialPortException {
      isConnected = false;
      serialPort.closePort();
   }
}