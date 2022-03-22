package us.ihmc.temperatureModel;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CurrentHeatProvider implements HeatValueProvider
{
   private final HeatableItem coilItem;
   private final double alpha;
   private final double resistance;
   private final double ambientResistorTemperature;
   private final CurrentProvider currentProvider;
   
   private final YoDouble heatFromCurrent;

   public CurrentHeatProvider(String prefix, HeatableItem coilItem, CurrentProvider currentProvider,
                              double alpha, double resistance, double ambientResistorTemperature,
                              YoRegistry registry)
   {
      this.coilItem = coilItem;
      this.currentProvider = currentProvider;
      this.alpha = alpha;
      this.resistance = resistance;
      this.ambientResistorTemperature = ambientResistorTemperature;
      heatFromCurrent = new YoDouble(prefix + "HeatFromCurrent", registry);
   }

   @Override
   public double getHeat()
   {
      return calculateHeatFromCurrent(this.currentProvider.getCurrent());
   }

   private double calculateHeatFromCurrent(double current)
   {
      double heatFromCurrentVal = (1 + alpha * (coilItem.getTemperature() - ambientResistorTemperature)) * resistance * (current * current);
      this.heatFromCurrent.set(heatFromCurrentVal);
      // see https://build-its-inprogress.blogspot.com/2019/
      return heatFromCurrentVal;
   }
}
