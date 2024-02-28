package us.ihmc.etherCAT.slaves.beckhoff;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * EL3104 Yo Wrapper - (Beckhoff 4 input Differential Analog Input)
 *
 * @author ur mom
 */
public class YoEL3104 implements VoltageSensor
{
   private final EL3104 el3104;
   private final YoInputWrapper[] inputs = new YoInputWrapper[4];
   private String name = getClass().getSimpleName();
   private YoRegistry registry = new YoRegistry(name);

   public YoEL3104(EL3104 el3104, YoRegistry parentRegistry)
   {
      this.el3104 = el3104;
      parentRegistry.addChild(registry);

      for (int i = 0; i < 4; i++)
      {
         inputs[i] = new YoInputWrapper(i, el3104.getInput(i), registry);
      }
   }

   public void read()
   {
      for (int i = 0; i < 4; i++)
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
      private final EL3104.Input input;
      private final YoBoolean underrange;
      private final YoBoolean overrange;
      private final YoBoolean error;
      private final YoBoolean state;
      private final YoBoolean toggle;
      private final YoDouble value;
      private final YoDouble valueInVolts;

      protected YoInputWrapper(int address, EL3104.Input input, YoRegistry registry)
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
         state.set(input.state.get());
         toggle.set(input.toggle.get());
         value.set(input.value.get());
         valueInVolts.set((input.value.get() / EL3104.resolution) * EL3104.referenceVoltage);
      }

      public double getVoltage()
      {
         return valueInVolts.getDoubleValue();
      }
   }

   ;
}
