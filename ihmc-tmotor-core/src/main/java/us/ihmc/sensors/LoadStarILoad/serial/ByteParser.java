package us.ihmc.sensors.LoadStarILoad.serial;


public interface ByteParser
{
   public abstract void parseByte(int i);
   public abstract double getForce();
}