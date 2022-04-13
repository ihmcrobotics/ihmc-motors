package us.ihmc.CAN;

import peak.can.basic.TPCANMsg;

public class CANTools
{
   public static double uint_to_double(int currentValue, double minValue, double maxValue, int bits)
   {
      double span = maxValue - minValue;
      double offset = minValue;
      return ((double) currentValue) * span / ((double) ((1 << bits) - 1)) + offset;
   }

   public static int double_to_uint(double currnetValue, double minValue, double maxValue, int bits)
   {
      double span = maxValue - minValue;
      if (currnetValue < minValue)
         currnetValue = minValue;
      else if (currnetValue > maxValue)
         currnetValue = maxValue;

      return (int) ((currnetValue - minValue) * ((double) (1 << bits) - 1) / span);
   }

   public static double motorToActuatorPosition(int position, int gearRatioToOne, int maxBitPosition, int currentMotorTurnCount)
   {
      return 2.0 * Math.PI / gearRatioToOne / maxBitPosition * position + 2.0 * Math.PI / gearRatioToOne * currentMotorTurnCount;
   }

   public static double motorToActuatorSpeed(double speed, int gearRatioToOne)
   {
      return speed * (Math.PI / 180.0) / gearRatioToOne;
   }

   public static int actuatorToMotorPosition(double desiredPosition, int gearRatioToOne, int encoderResolution)
   {
      return (int) (gearRatioToOne * (Math.toDegrees(desiredPosition)) * encoderResolution);
   }
   
   public static int getID(TPCANMsg message)
   {
      return message.getData()[0];
   }
}
