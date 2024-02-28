package us.ihmc.gyemsCore.parameters;

public class GyemsMotorParameters
{
   private final static int GEAR_RATIO_TO_ONE = 6;
   private final static int MAX_POSITION_BIT = 65536;           // 16-bit encoder reading unsigned int
   private final static int HALF_MAX_POSITION_BIT = 32768;
   private final static int HALF_MAX_TORQUE_BIT = 2048;        // 12-bits
   private final static int ENCODER_POSITION_RESOLUTION = 100; // 0.01 deg/LSB form GYEMS encoder
   private final static double MAX_CURRENT = 33.0;                // according to RMD CAN documentation

   public GyemsMotorParameters()
   {

   }

   public int getGearRatioToOne()
   {
      return GEAR_RATIO_TO_ONE;
   }

   public int getMaxPositionBit()
   {
      return MAX_POSITION_BIT;
   }

   public int getHalfMaxPositionBit()
   {
      return HALF_MAX_POSITION_BIT;
   }

   public int getHalfMaxTorqueBit()
   {
      return HALF_MAX_TORQUE_BIT;
   }

   public double getMaxCurrent()
   {
      return MAX_CURRENT;
   }

   public int getEncoderPositionResolution()
   {
      return ENCODER_POSITION_RESOLUTION;
   }
}
