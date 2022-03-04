package us.ihmc.temperatureModel;

import java.util.ArrayList;
import java.util.List;

public abstract class TemperatureModel
{
   protected TemperatureSimulator simulator = new TemperatureSimulator();

   public abstract double getTemperature();

   public void update(double dt) {
      simulator.stepByDt(dt);
   }

   protected class TemperatureSimulator
   {
      private List<HeatableItem> heatableItems = new ArrayList<HeatableItem>();
      private List<HeatRelation> heatRelations = new ArrayList<HeatRelation>();

      public void addHeatItem(HeatableItem heatItem)
      {
         heatableItems.add(heatItem);
      }

      public void addHeatRelation(HeatRelation heatRelation)
      {
         heatRelations.add(heatRelation);
      }

      public void stepByDt(double dt)
      {
         for (int i = 0; i < heatRelations.size(); i++)
         {
            heatRelations.get(i).moveHeat();
         }
         for (int i = 0; i < heatableItems.size(); i++)
         {
            heatableItems.get(i).applyHeatAtDt(dt);
         }
      }
   }


}
