package us.ihmc.sensors.LoadStarILoad;

import java.io.IOException;
import java.io.OutputStream;

import LoadStarILoad.serial.SerialPortTools;
import LoadStarILoad.settings.LoadStarILoadCommandEnum;

public class LoadStarILoadWriter
{
   private final OutputStream outputStream;

   public LoadStarILoadWriter(OutputStream outputStream)
   {
      this.outputStream = outputStream;
   }

   public void disconnect() throws IOException
   {
      outputStream.close();
   }

   public void sendCommand(LoadStarILoadCommandEnum commandEnum)
   {
      SerialPortTools.sendByteArray(commandEnum.getCommand(), outputStream);
   }
}
