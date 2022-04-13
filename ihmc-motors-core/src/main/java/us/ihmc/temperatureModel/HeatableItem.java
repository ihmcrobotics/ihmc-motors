package us.ihmc.temperatureModel;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class HeatableItem
{
   private double heatToAdd;
   private YoDouble heatToAddReporter;
   private YoDouble temperature;
   private final double thermalMass;

   public HeatableItem(String name, double initialTemperature, double thermalMass, YoRegistry registry)
   {
      heatToAddReporter = new YoDouble(name + "HeatToAdd", registry);
      heatToAddReporter.set(0);
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
      heatToAddReporter.set(heatToAdd);
      temperature.set(temperature.getDoubleValue() + heatToAdd / thermalMass * dt);
      heatToAdd = 0;
   }

}
