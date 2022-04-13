package us.ihmc.temperatureModel;

public class PowerSourceHeatRelation implements HeatRelation
{
   private final HeatableItem heatItem;
   private final HeatValueProvider heatProvider;

   public PowerSourceHeatRelation(HeatableItem heatItem, HeatValueProvider heatProvider)
   {
      this.heatItem = heatItem;
      this.heatProvider = heatProvider;
   }

   @Override
   public void moveHeat()
   {
      double heatToAdd = this.heatProvider.getHeat();
      this.heatItem.accumulateHeat(heatToAdd);
   }
}
