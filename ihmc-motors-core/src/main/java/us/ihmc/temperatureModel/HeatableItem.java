package us.ihmc.temperatureModel;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class HeatableItem
{
   private double heatToAdd;
   private YoDouble temperature;
   private final double thermalMass;

   public HeatableItem(String name, double initialTemperature, double thermalMass, YoRegistry registry)
   {
      heatToAdd = 0;
      temperature = new YoDouble(name + "Temperature", registry);
      temperature.set(initialTemperature);
      this.thermalMass = thermalMass;
   }

   public void setInitialTemperature(double initialTemperature)
   {
      temperature.set(initialTemperature);
   }

   public double getTemperature()
   {
      return temperature.getDoubleValue();
   }

   public void accumulateHeat(double heat)
   {
      heatToAdd += heat;
   }

   public void applyHeatAtDt(double dt)
   {
      temperature.set(temperature.getDoubleValue() + heatToAdd / thermalMass * dt);
      heatToAdd = 0;
   }

}
