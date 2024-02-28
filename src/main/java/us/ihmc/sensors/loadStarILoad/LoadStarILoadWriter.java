package us.ihmc.sensors.loadStarILoad;

import jssc.SerialPort;
import jssc.SerialPortException;
import us.ihmc.sensors.loadStarILoad.serial.SerialPortTools;
import us.ihmc.sensors.loadStarILoad.settings.LoadStarILoadCommandEnum;

public class LoadStarILoadWriter
{
   private final SerialPort serialPort;

   public LoadStarILoadWriter(SerialPort serialPort)
   {
      this.serialPort = serialPort;
   }

   public void disconnect() throws SerialPortException
   {
      serialPort.closePort();
   }

   public void sendCommand(LoadStarILoadCommandEnum commandEnum)
   {
      SerialPortTools.sendByteArray(commandEnum.getCommand(), serialPort);
   }
}
