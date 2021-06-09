package us.ihmc.sensors.LoadStarILoad.serial;

import java.io.IOException;
import java.io.InputStream;

public class SerialPortReader implements Runnable
{
   private final InputStream inputStream;
   private final ByteParser parser;
   private boolean isConnected = true;
   
   public SerialPortReader(InputStream inputStream, ByteParser parser)
   {
      this.inputStream = inputStream;
      this.parser = parser;
   }

   public void run()
   {
      int byteValue = -1;
      try{
         System.out.println("Starting Serial port reader thread");
         
         while (isConnected)
         {
            if ((byteValue = inputStream.read()) > -1)
            {
               parser.parseByte(byteValue);
            }
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
         throw new RuntimeException("Cannot read serial port");
      }
   }

   public void disconnect() throws IOException
   {
      isConnected = false;
      inputStream.close();
   }
}