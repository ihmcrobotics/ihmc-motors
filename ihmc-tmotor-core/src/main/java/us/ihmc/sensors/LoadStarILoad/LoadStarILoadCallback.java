package us.ihmc.sensors.LoadStarILoad;

import us.ihmc.tools.*;

public class LoadStarILoadCallback
{
   private static final double MILLIPOUND_TO_NEWTON = 4.4482216283 / 1000.0;
   private double forceNewton;
   private double forceMilliPounds;

   public void doPing(int[] byteBuffer, int lengthOfValidData)
   {
      if (!LoadstarILoadByteManipulationTools.isAcknowledgement(byteBuffer, lengthOfValidData))
         throw new RuntimeException("LoadStarILoadCallback: Bad ping!");
      else
         System.out.println("Received ping");
   }

   public void doTare(int[] byteBuffer, int lengthOfValidData)
   {
      if (!LoadstarILoadByteManipulationTools.isAcknowledgement(byteBuffer, lengthOfValidData))
         throw new RuntimeException("LoadStarILoadCallback: Bad tare!");
      else
         System.out.println("Loadstar ILoad tared.");
   }

   public void doWeight(int[] byteBuffer, int lengthOfValidData)
   {

      if (lengthOfValidData != LoadstarILoadByteManipulationTools.DO_WEIGHT_BUFFER_LENGTH)
         throw new RuntimeException("LoadStarILoadCallback: Bad weight data! lengthOfValidData: " + lengthOfValidData);
      forceMilliPounds = LoadstarILoadByteManipulationTools.loadStarByteArrayToInt(byteBuffer);
      forceNewton = forceMilliPounds * MILLIPOUND_TO_NEWTON;


      sleep(1L);
   }

   public double getForceNewton()
   {
      return forceNewton;
   }

   public double getForcePound() {
      return forceMilliPounds / 1000;
   }

   private void sleep(long millis)
   {
      try
      {
         Thread.sleep(millis);
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }
}
