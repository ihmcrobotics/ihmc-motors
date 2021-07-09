package us.ihmc.sensors.footsole;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.HashMap;

public class YoEtherCATPressureSensor
{
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

    private static final int NUMBER_OF_SENSORS = 8;
    private final EtherCATPressureSensor pressureSensor;
    private final YoInteger readLoopDuration = new YoInteger("readLoopDuration", registry);
    private final HashMap<Integer, YoInteger> pressureMap = new HashMap<>();
    private double[] measuredNormalForces = new double[NUMBER_OF_SENSORS];

    private final YoDouble x0, x1, x2;

    public YoEtherCATPressureSensor(EtherCATPressureSensor pressureSensor, YoRegistry parentRegistry)
    {
        this.pressureSensor = pressureSensor;

        for(int i = 0; i < NUMBER_OF_SENSORS; i++)
        {
            pressureMap.put(i, new YoInteger("pressureMBar" + i, registry));
        }

        x0 = new YoDouble("pressureSensor_x0", registry);
        x1 = new YoDouble("pressureSensor_x1", registry);
        x2 = new YoDouble("pressureSensor_x2", registry);

        parentRegistry.addChild(registry);
    }

    public void setCoeffs(double x0, double x1, double x2)
    {
        this.x0.set(x0);
        this.x1.set(x1);
        this.x2.set(x2);
    }

    public void read()
    {
        for(int i = 0; i < NUMBER_OF_SENSORS; i++)
        {
            pressureMap.get(i).set(pressureSensor.getPressureMbar(i));
        }
        readLoopDuration.set(pressureSensor.getDuration().get());
    }

    public double[] getMeasuredNormalForces()
    {
        for(int i = 0; i < NUMBER_OF_SENSORS; i++)
        {
            int currentPressureReading = pressureMap.get(i).getIntegerValue();
            // normalForce = currentPressureReading * area;
            double normalForce = x2.getDoubleValue() * currentPressureReading * currentPressureReading +
                    x1.getDoubleValue() * currentPressureReading + x0.getDoubleValue();
            measuredNormalForces[i] = normalForce;
        }

        return measuredNormalForces;
    }
}
