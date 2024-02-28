package us.ihmc.sensors.loadStarILoad.serial;

public interface ByteParser
{
   public abstract void parseByte(int i);

   public abstract double getForce();
}