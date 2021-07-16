package us.ihmc.sensors.LoadStarILoad.serial;

import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import jssc.SerialPort;
import jssc.SerialPortException;
import org.lwjgl.Sys;
import us.ihmc.sensors.LoadStarILoad.LoadstarILoadByteManipulationTools;

import java.io.IOException;

public class SerialPortReader implements Runnable
{
   private final SerialPort serialPort;
   private final ByteParser parser;
   private boolean isConnected = true;
   private Double force;
   private StringProperty forceString = new SimpleStringProperty("");

   public SerialPortReader(SerialPort serialPort, ByteParser parser) {
      this.serialPort = serialPort;
      this.parser = parser;
   }

   public void run()
   {
      try{
         System.out.println("Starting Serial port reader thread");

         if(isConnected)
         {
            int [] byteArray = serialPort.readIntArray();
            if (byteArray != null) {
               for(int byteValue : byteArray)
                  parser.parseByte(byteValue);
               force = parser.getForce();
               forceString.set(force.toString());
               System.out.println(forceString);
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