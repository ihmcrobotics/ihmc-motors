package us.ihmc.temperatureModel;

public class CurrentHeatProvider implements HeatValueProvider
{
   private final HeatableItem coilItem;
   private final double alpha;
   private final double resistance;
   private final double ambientResistorTemperature;
   private final CurrentProvider currentProvider;

   public CurrentHeatProvider(HeatableItem coilItem, CurrentProvider currentProvider,
                              double alpha, double resistance, double ambientResistorTemperature)
   {
      this.coilItem = coilItem;
      this.currentProvider = currentProvider;
      this.alpha = alpha;
      this.resistance = resistance;
      this.ambientResistorTemperature = ambientResistorTemperature;
   }

   @Override
   public double getHeat()
   {
      return calculateHeatFromCurrent(this.currentProvider.getCurrent());
   }

   private double calculateHeatFromCurrent(double current)
   {
      current += 0.008;
      // see https://build-its-inprogress.blogspot.com/2019/
      return (1 + alpha * (coilItem.getTemperature() - ambientResistorTemperature)) * resistance * current * current;
   }
}
