package us.ihmc.sensors.loadStarILoad.settings;

import us.ihmc.sensors.loadStarILoad.serial.ByteManipulationTools;

public enum LoadStarILoadCommandEnum
{
   PING("\r"),
   TARE("CT0\r"),
   OUTPUT_WEIGHT_ONCE("O0W1\r"),
   OUTPUT_WEIGHT_CONTINUOUSLY("O0W0\r"),
   OUTPUT_TEMPERATURE("O0S2\r"),
   OUTPUT_LOAD_CAPACITY("SLC\r"),
   OUTPUT_SERIAL_NUMBER("SS1\r"),
   OUTPUT_MODEL_NUMBER("SS0\r");

   private final int[] command;

   private LoadStarILoadCommandEnum(String command)
   {
      this.command = ByteManipulationTools.stringToByteArray(command);
   }

   public int[] getCommand()
   {
      return command;
   }
}