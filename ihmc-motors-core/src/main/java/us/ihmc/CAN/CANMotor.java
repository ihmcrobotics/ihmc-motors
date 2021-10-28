package us.ihmc.CAN;

import peak.can.basic.TPCANHandle;
import peak.can.basic.TPCANMsg;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class CANMotor
{
   protected final String name = getClass().getSimpleName();
   protected final YoRegistry registry;

   // PCAN fields
   protected TPCANHandle channel;
   protected final TPCANMsg receivedMsg = new TPCANMsg();
   protected final int ID;
   protected final String motorName;

   // inputs
   protected final YoDouble measuredEncoderPosition;
   protected final YoDouble measuredActuatorPosition;
   protected final YoDouble measuredVelocity;

   protected final YoDouble measuredTorque;
   protected final AlphaFilteredYoVariable filteredTorque;
   protected final YoDouble velocityFilterCoefficient;

   protected final FilteredVelocityYoVariable filteredVelocity;

   protected final YoInteger motorDirection;

   // debug
   protected final YoCANMsg yoCANMsg;

   public CANMotor(int id, String motorName, double dt)
   {
      this.ID = id;
      this.motorName = motorName;
      String prefix = motorName + "_";

      registry = new YoRegistry(prefix + name);
      yoCANMsg = new YoCANMsg(motorName, registry);

      measuredEncoderPosition = new YoDouble(prefix + "measuredEncoderPosition", registry); // encoder tick value
      measuredActuatorPosition = new YoDouble(prefix + "measuredActuatorPosition", registry); // rad
      measuredVelocity = new YoDouble(prefix + "measuredVelocity", registry); // rad/sec

      measuredTorque = new YoDouble(prefix + "measuredTorque", registry);
      filteredTorque = new AlphaFilteredYoVariable(prefix + "measuredTorqueFiltered", registry, 0.9, measuredTorque);
      velocityFilterCoefficient = new YoDouble(prefix + "velocityFilterCoefficient", registry);

      filteredVelocity = new FilteredVelocityYoVariable(prefix + "filteredVelocity", null, velocityFilterCoefficient, measuredActuatorPosition, dt, registry);
      motorDirection = new YoInteger(prefix + "motorDirection", registry);
   }

   /**
    * Read can message and store it in YoVariable
    *
    * @param message CAN message using the PCAN miniPCIe card
    */
   public abstract void read(TPCANMsg message);

   public abstract void update();

   public YoCANMsg getYoCANMsg()
   {
      return this.yoCANMsg;
   }

   public int getID()
   {
      return ID;
   }
}