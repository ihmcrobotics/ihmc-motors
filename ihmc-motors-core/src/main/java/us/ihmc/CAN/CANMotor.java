package us.ihmc.CAN;

import peak.can.basic.PCANBasic;
import peak.can.basic.TPCANHandle;
import peak.can.basic.TPCANMessageType;
import peak.can.basic.TPCANMsg;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class CANMotor
{
   protected final String name = getClass().getSimpleName();
   protected final YoRegistry registry;

   // CAN stuff
   protected PCANBasic canBus;
   protected TPCANHandle channel;
   protected final TPCANMsg receivedMsg = new TPCANMsg();
   protected final int ID;
   protected final String motorName;

   // inputs
   protected final YoDouble measuredEncoderPosition;
   protected final YoDouble measuredActuatorPosition;
   protected final YoDouble measuredVelocity;

   protected final YoDouble measuredTorqueCurrent;
   protected final YoDouble velocityFilterCoefficient;

   protected final FilteredVelocityYoVariable filteredVelocity;

   // debug
   protected final YoCANMsg yoCANMsg;

   protected static final byte STANDARD_CAN_MESSAGE = TPCANMessageType.PCAN_MESSAGE_STANDARD.getValue();
//   protected final YoFunctionGenerator functionGenerator;

   public CANMotor(int id, String motorName, double dt)
   {
      this.ID = id;
      this.motorName = motorName;
      String prefix = motorName +"_";

      registry = new YoRegistry(prefix + name);
      yoCANMsg = new YoCANMsg(motorName, registry);

      measuredEncoderPosition = new YoDouble(prefix + "measuredEncoderPosition", registry); // encoder tick value
      measuredActuatorPosition = new YoDouble(prefix + "measuredActuatorPosition", registry); // rad
      measuredVelocity = new YoDouble(prefix + "measuredVelocity", registry); // rad/sec

      measuredTorqueCurrent = new YoDouble(prefix + "measuredTorqueCurrent", registry);
      velocityFilterCoefficient = new YoDouble(prefix + "velocityFilterCoefficient", registry);

      filteredVelocity = new FilteredVelocityYoVariable(prefix + "filteredVelocity", null, velocityFilterCoefficient, measuredActuatorPosition, dt, registry);

//      functionGenerator = new YoFunctionGenerator(prefix + "functionGenerator", time, registry);
//      functionGenerator.setAlphaForSmoothing(0.99);
   }

   /**
    * Read can message and store it in YoVariable
    *
    * @param message CAN message using the PCAN miniPCIe card
    */
   public abstract void read(TPCANMsg message);

   public abstract void update();

   public abstract TPCANMsg write();

   public abstract void setCanBus(PCANBasic canBus, TPCANHandle channel);

   public YoCANMsg getYoCANMsg() {return this.yoCANMsg;}
}
