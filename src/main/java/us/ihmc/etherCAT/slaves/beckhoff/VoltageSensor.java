package us.ihmc.etherCAT.slaves.beckhoff;

public interface VoltageSensor
{
   public double getVoltageForChannel(int channel);
}
