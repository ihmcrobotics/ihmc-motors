package us.ihmc.etherCAT.slaves.beckhoff;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoAnalogSignalWrapper
{
   private final YoRegistry registry;
   
   private final YoDouble x0, x1, x2;
   private final YoDouble result;
   private final VoltageSensor sensor;
   private final int channel;
   
   public YoAnalogSignalWrapper(String signalName, VoltageSensor sensor, int channel, YoRegistry parentRegistry)
   {
      this.sensor = sensor;
      this.channel = channel;
      
      registry = new YoRegistry(signalName);
      
      x0 = new YoDouble(signalName + "_x0", registry);
      x1 = new YoDouble(signalName + "_x1", registry);
      x2 = new YoDouble(signalName + "_x2", registry);
      result = new YoDouble(signalName + "_result", registry);
      
      parentRegistry.addChild(registry);
   }
   
   public void setCoeffs(double x0, double x1, double x2)
   {
      this.x0.set(x0);
      this.x1.set(x1);
      this.x2.set(x2);
   }
   
   public void update()
   {
      double voltage = sensor.getVoltageForChannel(channel);
      result.set(x2.getDoubleValue() * Math.pow(voltage, 2) + x1.getDoubleValue() * voltage + x0.getDoubleValue());
   }
   
   public YoDouble getResultYoVariable()
   {
      return result;
   }
}
