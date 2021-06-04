package us.ihmc.CAN;

import peak.can.basic.TPCANMessageType;
import peak.can.basic.TPCANMsg;
import us.ihmc.CAN.YoCANMsg;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class CANMotor
{
   protected final String name = getClass().getSimpleName();
   protected final YoRegistry registry;
   protected final int ID;

   // inputs
   protected final YoDouble measuredEncoderPosition;
   protected final YoDouble measuredActuatorPosition;
   protected final YoDouble measuredVelocity;

   protected final YoDouble measuredTorqueCurrent;
   protected final YoDouble velocityFilterCoefficient;
   protected final YoDouble desiredActuatorPosition;
   protected final YoDouble desiredActuatorVelocity;
   protected final YoDouble desiredActuatorTorque;

   protected final FilteredVelocityYoVariable filteredVelocity;

   // debug
   protected final YoCANMsg yoCANMsg;
   protected static final byte STANDARD_CAN_MESSAGE = TPCANMessageType.PCAN_MESSAGE_STANDARD.getValue();
   protected final YoFunctionGenerator functionGenerator;

   public CANMotor(int ID, double dt, YoDouble time, YoRegistry parentRegistry)
   {
      this.ID = ID;
      String prefix = ID +"_";

      registry = new YoRegistry(prefix + name);
      yoCANMsg = new YoCANMsg(prefix, ID, registry);
      
      measuredEncoderPosition = new YoDouble(prefix + "measuredEncoderPosition", registry); // encoder tick value        
      measuredActuatorPosition = new YoDouble(prefix + "measuredActuatorPosition", registry); // rad                     
      measuredVelocity = new YoDouble(prefix + "measuredVelocity", registry); // rad/sec                                 
                                                                                                                
      measuredTorqueCurrent = new YoDouble(prefix + "measuredTorqueCurrent", registry);                                  
      velocityFilterCoefficient = new YoDouble(prefix + "velocityFilterCoefficient", registry);                          
      desiredActuatorPosition = new YoDouble(prefix + "desiredActuatorPosition", registry);                              
      desiredActuatorVelocity = new YoDouble(prefix + "desiredActuatorVelocity", registry);
      desiredActuatorTorque = new YoDouble(prefix + "desiredActuatorTorque", registry);

      filteredVelocity = new FilteredVelocityYoVariable(prefix + "filteredVelocity", null, velocityFilterCoefficient, measuredActuatorPosition, dt, registry);

      functionGenerator = new YoFunctionGenerator(prefix + "functionGenerator", time, registry);
      functionGenerator.setAlphaForSmoothing(0.99);
   }

   /**
    * Read can message and store it in YoVariable
    * 
    * @param message CAN message using the PCAN miniPCIe card
    */
   public abstract void read(TPCANMsg message);

   public abstract void update();

   public abstract TPCANMsg write();
}
