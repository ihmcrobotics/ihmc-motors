package us.ihmc.sensors.LoadStarILoad;

public class LoadstarILoadByteManipulationTools
{
   private static final int ACKNOWLEDGE_BYTE = 'A';
   private static final int NULL_BYTE = 0x0;
   private static final int SPACE_BYTE = ' ';
   private static final int MINUS_BYTE = '-';
   private static final int ASCII_START_OF_NUMBERS = 48;
   public static final int DO_WEIGHT_BUFFER_LENGTH = 9; //was 12, didn't work until we changed to 9

   public static boolean isAcknowledgement(int[] byteBuffer, int lengthOfValidData)
   {
      return (lengthOfValidData == 1) && (byteBuffer[0] == ACKNOWLEDGE_BYTE);
   }

   public static int loadStarByteArrayToInt(int[] byteBuffer)
   {
      int startIndex = 0;
      while ((byteBuffer[startIndex] == NULL_BYTE) || (byteBuffer[startIndex] == SPACE_BYTE))
      {
         startIndex++;
      }

      int ret = 0;
      int sign = 1;
      for (int i = startIndex; i < DO_WEIGHT_BUFFER_LENGTH; i++)
      {
         int byteValue = byteBuffer[i];
         if (byteValue == MINUS_BYTE)
            sign = -1;
         else
         {
            int intValue = byteValue - ASCII_START_OF_NUMBERS;
            int decimal = DO_WEIGHT_BUFFER_LENGTH - i - 1;
            ret += intValue * Math.pow(10, decimal);
         }
      }

      ret *= sign;

      return ret;
   }
}
