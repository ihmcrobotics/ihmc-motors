package us.ihmc.sensors.loadStarILoad;

import us.ihmc.sensors.loadStarILoad.serial.ByteParser;
import us.ihmc.sensors.loadStarILoad.settings.LoadStarILoadCommandEnum;

public class LoadStarILoadParser implements ByteParser
{
   private final LoadStarILoadCallback loadStarILoadCallback;
   private final int[] byteBuffer = new int[LoadStarILoadByteManipulationTools.DO_WEIGHT_BUFFER_LENGTH];
   private LoadStarILoadCommandEnum expectedCommand;
   private int byteBufferIndex = 0;

   public LoadStarILoadParser(LoadStarILoadCallback loadStarILoadCallback)
   {
      this.loadStarILoadCallback = loadStarILoadCallback;
   }

   public void parseByte(int i)
   {

      if (isNewLine(i))
         return;    // don't know why these get sent

      if ((expectedCommand == null) || isNull(i))
      {
         clearBuffer();
      }
      else if (isEndOfLine(i))
      {
         switch (expectedCommand)
         {
            case PING:
               loadStarILoadCallback.doPing(byteBuffer, byteBufferIndex);

               break;

            case TARE:
               loadStarILoadCallback.doTare(byteBuffer, byteBufferIndex);

               break;

            case OUTPUT_WEIGHT_ONCE:
               loadStarILoadCallback.doWeight(byteBuffer, byteBufferIndex);

               break;

            case OUTPUT_WEIGHT_CONTINUOUSLY:
               loadStarILoadCallback.doWeight(byteBuffer, byteBufferIndex);

               break;

            case OUTPUT_LOAD_CAPACITY:
               System.out.println("Implement me!");

               break;

            case OUTPUT_MODEL_NUMBER:
               System.out.println("Implement me!");

               break;

            case OUTPUT_SERIAL_NUMBER:
               System.out.println("Implement me!");

               break;

            case OUTPUT_TEMPERATURE:
               System.out.println("Implement me!");

               break;

            default:
               break;
         }
      }
      else
      {
         storeInBuffer(i);
      }
   }

   private void clearBuffer()
   {
      byteBufferIndex = 0;
   }

   private void storeInBuffer(int i)
   {
      byteBuffer[byteBufferIndex] = i;
      byteBufferIndex++;
   }

   private boolean isEndOfLine(int i)
   {
      return i == 0xD;
   }

   private boolean isNewLine(int i)
   {
      return i == 0xA;
   }

   private boolean isNull(int i)
   {
      return i == 0;
   }

   public void expectCommand(LoadStarILoadCommandEnum command)
   {
      this.expectedCommand = command;
   }

   public double getForce()
   {
      return loadStarILoadCallback.getForceNewton();
   }
}
