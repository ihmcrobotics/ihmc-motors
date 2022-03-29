package us.ihmc.etherCAT.slaves.beckhoff;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * YoEL3062 Yo Wrapper - (Beckhoff 2 input Single Ended Analog Input)
 * @author ur mom
 *
 */
public class YoEL3062 implements VoltageSensor
{
   private String name = getClass().getSimpleName();
   private YoRegistry registry = new YoRegistry(name);
   private final EL3062 el3062;
   private final YoInputWrapper[] inputs = new YoInputWrapper[2];
   
   public YoEL3062(EL3062 el3062, YoRegistry parentRegistry)
   {
      this.el3062 = el3062;
      parentRegistry.addChild(registry);
      
      for(int i = 0; i < inputs.length; i++)
      {
         inputs[i] = new YoInputWrapper(i, el3062.getInput(i), registry);
      }
   }
   
   public void read()
   {
      for(int i = 0; i < inputs.length; i++)
      {
         inputs[i].read();
      }
   }
   
   public double getVoltageForChannel(int channel)
   {
      return inputs[channel].getVoltage();
   }
   
   private class YoInputWrapper 
   {
      private final EL3062.AnalogInput input;
      private final YoBoolean underrange;
      private final YoBoolean overrange;
      private final YoBoolean error;
      private final YoBoolean state;
      private final YoBoolean toggle;
      private final YoDouble  value;
      private final YoDouble  valueInVolts;
      
      protected YoInputWrapper(int address, EL3062.AnalogInput input, YoRegistry registry)
      {
         this.input = input;
         String prefix = "channel" + address + "_";
         
         underrange = new YoBoolean(prefix + "underrange", registry);
         overrange = new YoBoolean(prefix + "overrange", registry);
         error = new YoBoolean(prefix + "error", registry);
         state = new YoBoolean(prefix + "state", registry);
         toggle = new YoBoolean(prefix + "toggle", registry);
         value = new YoDouble(prefix + "value", registry);
         valueInVolts = new YoDouble(prefix + "valueInVolts", registry);
      }

      public void read()
      {
         underrange.set(input.underrange.get());
         overrange.set(input.overrange.get()); 
         error.set(input.error.get());
         state.set(input.txPdoState.get());     
         toggle.set(input.txPdoToggle.get());    
         value.set(input.value.get());     
         valueInVolts.set((input.value.get() / EL3062.RESOLUTION) * EL3062.MAX_VOLTAGE);     
      }
      
      public double getVoltage()
      {
         return valueInVolts.getDoubleValue();
      }
   };
}
