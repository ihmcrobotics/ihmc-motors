package us.ihmc.temperatureModel;

public class ConductionHeatRelation implements HeatRelation
{
   private final HeatableItem heatItem1;
   private final HeatableItem heatItem2;
   private final double conductivity;

   public ConductionHeatRelation(HeatableItem heatItem1, HeatableItem heatItem2, double conductivity) {
      this.heatItem1 = heatItem1;
      this.heatItem2 = heatItem2;
      this.conductivity = conductivity;
   }

   @Override
   public void moveHeat()
   {
      double temp1 = this.heatItem1.getTemperature();
      double temp2 = this.heatItem2.getTemperature();

      // dQ from 1->2 = k (T1 - T2)
      double heatToMove12 = this.conductivity * (temp1 - temp2);
      this.heatItem1.accumulateHeat(-heatToMove12);
      this.heatItem2.accumulateHeat(heatToMove12);
   }
}
