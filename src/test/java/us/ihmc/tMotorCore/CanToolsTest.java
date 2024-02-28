package us.ihmc.tMotorCore;

import org.junit.jupiter.api.Test;
import us.ihmc.can.CANTools;

import static org.junit.jupiter.api.Assertions.*;

public class CanToolsTest
{
   @Test
   public void testDoubleToRawAndBack()
   {
      double max = Math.random() * 100.0;
      double min = -max;
      for (int bits = 8; bits < 16; bits++)
      {
         for (int i = 0; i < 10000; i++)
         {
            double value = Math.random() * max * 2.0 - max;
            int test = CANTools.double_to_uint(value, min, max, bits);
            double result = CANTools.uint_to_double(test, min, max, bits);

            double precision = (max - min) / Math.pow(2.0, bits) * 1.01;
            if (Math.abs(value - result) > precision)
            {
               fail("Expected: " + value + " Result:" + result + " bits: " + bits + " precision: " + precision);
            }
         }
      }
   }
}
