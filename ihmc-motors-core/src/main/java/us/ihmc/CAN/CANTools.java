package us.ihmc.CAN;

public class CANTools
{
   public static float uint_to_float(int currentValue, float minValue, float maxValue, int bits)
   {
      float span = maxValue - minValue;
      float offset = minValue;
      return ((float) currentValue) * span / ((float) ((1 << bits) - 1)) + offset;
   }

   public static int float_to_uint(float currnetValue, float minValue, float maxValue, int bits)
   {
      float span = maxValue - minValue;
      if (currnetValue < minValue)
         currnetValue = minValue;
      else if (currnetValue > maxValue)
         currnetValue = maxValue;

      return (int) ((currnetValue - minValue) * ((float) (1 << bits) - 1) / span);
   }

   public static double motorToActuatorPosition(int position, int gearRatioToOne, int maxBitPosition, int currentMotorTurnCount)
   {
      return 2.0*Math.PI/gearRatioToOne/maxBitPosition * position + 2.0*Math.PI/gearRatioToOne*currentMotorTurnCount;
   }

   public static double motorToActuatorSpeed(double speed, int gearRatioToOne)
   {
      return speed * (Math.PI/180.0) / gearRatioToOne;
   }

   public static int actuatorToMotorPosition(double desiredPosition, int gearRatioToOne, int encoderResolution)
   {
      return (int) (gearRatioToOne * (Math.toDegrees(desiredPosition)) * encoderResolution);
   }
}
