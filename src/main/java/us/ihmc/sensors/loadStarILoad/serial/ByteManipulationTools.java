package us.ihmc.sensors.loadStarILoad.serial;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;

public class ByteManipulationTools
{
   /*
    * From bytes
    */
   public static float byteArrayToFloat(int[] payload, int start)
   {
      if (payload.length < start + 3)
         throw new ArrayIndexOutOfBoundsException("Tried to access elements outside payload length");

      int bits = payload[start] << 24;
      bits += payload[start + 1] << 16;
      bits += payload[start + 2] << 8;
      bits += payload[start + 3];

      return Float.intBitsToFloat(bits);
   }

   public static int byteArrayToShort(int[] payload, int start)
   {
      if (payload.length < start + 1)
         throw new ArrayIndexOutOfBoundsException("Tried to access elements outside payload length");

      short bits = (short) (payload[start] << 8);
      bits += payload[start + 1];

      return bits;
   }

   public static long byteArrayToLong(int[] payload, int start)
   {
      if (payload.length < start + 4)
         throw new ArrayIndexOutOfBoundsException("Tried to access elements outside payload length");

      long bits = payload[start] << 24;
      bits += payload[start + 1] << 16;
      bits += payload[start + 2] << 8;
      bits += payload[start + 3];

      return bits;
   }

   public static Vector3D byteArrayToVector3d(int[] payload, int start)
   {
      float x = byteArrayToFloat(payload, start);
      float y = byteArrayToFloat(payload, start + 4);
      float z = byteArrayToFloat(payload, start + 8);

      return new Vector3D(x, y, z);
   }

   public static float[] byteArrayToFloat4d(int[] payload, int start)
   {
      float q1 = byteArrayToFloat(payload, start);
      float q2 = byteArrayToFloat(payload, start + 4);
      float q3 = byteArrayToFloat(payload, start + 8);
      float q4 = byteArrayToFloat(payload, start + 12);

      return new float[] {q1, q2, q3, q4};
   }

   public static Matrix3D byteArrayToMatrix3d(int[] payload, int start)
   {
      float m11 = byteArrayToFloat(payload, start);
      float m12 = byteArrayToFloat(payload, start + 4);
      float m13 = byteArrayToFloat(payload, start + 8);
      float m21 = byteArrayToFloat(payload, start + 12);
      float m22 = byteArrayToFloat(payload, start + 16);
      float m23 = byteArrayToFloat(payload, start + 20);
      float m31 = byteArrayToFloat(payload, start + 24);
      float m32 = byteArrayToFloat(payload, start + 28);
      float m33 = byteArrayToFloat(payload, start + 32);

      return new Matrix3D(m11, m12, m13, m21, m22, m23, m31, m32, m33);
   }

   public static RotationMatrix byteArrayToRotationMatrix(int[] payload, int start)
   {
      float m11 = byteArrayToFloat(payload, start);
      float m12 = byteArrayToFloat(payload, start + 4);
      float m13 = byteArrayToFloat(payload, start + 8);
      float m21 = byteArrayToFloat(payload, start + 12);
      float m22 = byteArrayToFloat(payload, start + 16);
      float m23 = byteArrayToFloat(payload, start + 20);
      float m31 = byteArrayToFloat(payload, start + 24);
      float m32 = byteArrayToFloat(payload, start + 28);
      float m33 = byteArrayToFloat(payload, start + 32);

      return new RotationMatrix(m11, m12, m13, m21, m22, m23, m31, m32, m33);
   }

   public static int[] byteArrayToInt4d(int[] payload, int start)
   {
      int e1 = byteArrayToShort(payload, start);
      int e2 = byteArrayToShort(payload, start + 2);
      int e3 = byteArrayToShort(payload, start + 4);
      int e4 = byteArrayToShort(payload, start + 6);

      return new int[] {e1, e2, e3, e4};
   }

   /*
    * To bytes
    */
   public static int bitArray8ToByte(boolean[] bitArray)
   {
      if (bitArray.length != Byte.SIZE)
         throw new RuntimeException("Bit array must have length" + Byte.SIZE + ". Length = " + bitArray.length);
      int ret = 0;
      int significance = 1;
      for (int i = Byte.SIZE - 1; i >= 0; i--)
      {
         int intValue = bitArray[i] ? 1 : 0;
         ret += intValue * significance;
         significance *= 2;
      }

      return ret;
   }

   public static int[] shortToByteArray(int i)
   {
      int b[] = new int[2];
      b[0] = ((i >>> 8) & 0xFF);
      b[1] = (i & 0xFF);

      return b;
   }

   public static int[] unsignedIntToByteArray(long value, int nBytes)
   {
      MathTools.checkIntervalContains(value, 0L, (long) Math.pow(2, nBytes * Byte.SIZE) - 1);

      int[] ret = new int[nBytes];
      for (int i = 0; i < nBytes; i++)
      {
         int rightShift = (nBytes - i - 1) * Byte.SIZE;
         ret[i] = (int) ((value >>> rightShift) & 0xFF);
      }

      return ret;
   }

   public static int[] intToByteArray(int i)
   {
      int b[] = new int[4];

      b[0] = ((i >>> 24) & 0xFF);
      b[1] = ((i >>> 16) & 0xFF);
      b[2] = ((i >>> 8) & 0xFF);
      b[3] = (i & 0xFF);

      return b;
   }

   public static int[] floatToByteArray(float f)
   {
      int i = Float.floatToIntBits(f);

      return intToByteArray(i);
   }

   public static int[] stringToByteArray(String s)
   {
      int[] ret = new int[s.length()];
      for (int i = 0; i < s.length(); i++)
      {
         ret[i] = s.charAt(i);
      }
      return ret;
   }
}
