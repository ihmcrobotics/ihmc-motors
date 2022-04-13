package us.ihmc.tMotorCore;

import us.ihmc.tMotorCore.parameters.TMotorParameters;
import us.ihmc.temperatureModel.*;
import us.ihmc.yoVariables.registry.YoRegistry;

public class TMotorTemperatureModel extends TemperatureModel
{
   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final HeatableItem coilHeatItem;
   private final HeatableItem motorBulkHeatItem;
   private final HeatableItem environmentHeatItem;
   private final ConductionHeatRelation motorCoilConduction;
   private final ConductionHeatRelation envMotorConduction;
   private final CurrentHeatProvider currentHeatProvider;
   private final PowerSourceHeatRelation currentPowerSource;

   public TMotorTemperatureModel(String prefix, TMotorParameters parameters, CurrentProvider currentProvider, YoRegistry parentRegistry)
   {
      double ambientTemperature = parameters.getDefaultAmbientTemperature();
      this.coilHeatItem = new HeatableItem(prefix + "Coil", ambientTemperature, parameters.getCoilThermalMass(), registry);
      this.motorBulkHeatItem = new HeatableItem(prefix + "MotorBulk", ambientTemperature, parameters.getMotorThermalMass(), registry);
      this.environmentHeatItem = new HeatableItem(prefix + "Environment", ambientTemperature, parameters.getEnvironmentThermalMass(), registry);

      this.motorCoilConduction = new ConductionHeatRelation(this.coilHeatItem, this.motorBulkHeatItem, parameters.getMotorCoilConductivity());
      this.envMotorConduction = new ConductionHeatRelation(this.motorBulkHeatItem, this.environmentHeatItem, parameters.getEnvMotorConductivity());
      this.currentHeatProvider = new CurrentHeatProvider(prefix, this.coilHeatItem, currentProvider,
                                                         parameters.getCurrentAlpha(), parameters.getElectricalResistance(), parameters.getAmbientResistorTemperature(),
                                                         registry);
      this.currentPowerSource = new PowerSourceHeatRelation(this.coilHeatItem, this.currentHeatProvider);

      this.simulator.addHeatItem(this.coilHeatItem);
      this.simulator.addHeatItem(this.motorBulkHeatItem);
      this.simulator.addHeatItem(this.environmentHeatItem);
      this.simulator.addHeatRelation(this.motorCoilConduction);
      this.simulator.addHeatRelation(this.envMotorConduction);
      this.simulator.addHeatRelation(this.currentPowerSource);

      parentRegistry.addChild(registry);
   }

   @Override
   public double getTemperature()
   {
      return coilHeatItem.getTemperature();
   }

   public void resetTemperaturesTo(double temperature)
   {
      coilHeatItem.setInitialTemperature(temperature);
      motorBulkHeatItem.setInitialTemperature(temperature);
   }
}
