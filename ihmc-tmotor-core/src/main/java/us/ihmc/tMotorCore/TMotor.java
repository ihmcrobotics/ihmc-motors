package us.ihmc.tMotorCore;

import peak.can.basic.TPCANMsg;
import us.ihmc.CAN.YoCANMsg;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.tMotorCore.CANMessages.TMotorCommand;
import us.ihmc.tMotorCore.CANMessages.TMotorReply;
import us.ihmc.tMotorCore.parameters.TMotorParameters;
import us.ihmc.temperatureModel.CurrentProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.function.BooleanSupplier;

/**
 * This class helps control a TMotor using the PCAN Can hardware and library This has been tested
 * using: https://www.peak-system.com/PCAN-PCI-Express.206.0.html?&L=1 The methods provide access to
 * CAN messages defined in https://store.cubemars.com/images/file/20211201/1638329381542610.pdf
 */
public class TMotor
{
   public static boolean ENABLE_DEBUG_VARIABLES = true;
   private final YoRegistry registry;
   private final YoRegistry debugRegistry;

   // PCAN fields
   private final int ID;
   private final String motorName;
   private final TMotorCommand motorCommand; // CAN message sent to motor
   private final TMotorReply motorReply; // CAN message reply from motor
   private final YoCANMsg yoCANMsg;
   private final TMotorVersion version;

   /**
    * The encoder is on the input side, and is clipped to (-pi,pi) on boot. So on the output side, the
    * joint position might be off by a factor of 2*pi/gearRatio if it boots in the wrong sector. So
    * this can be used to re-zero the joint online.
    */
   private final YoInteger offsetIntervalRequested;
   private final RateLimitedYoVariable offsetIntervalRateLimited;
   private final double outputAnglePerInputRevolution;
   private final YoInteger motorDirection;

   /**
    * The T-Motor firmware uses at Kt=0.095 Nm/A, but this is under-approximated. This multiplier can
    * be used to adjust to the actual Kt. Desired torques will be scaled down by this value before
    * being sent to the motor controller and the motor controller's reported torque will be scaled up
    * by this value.
    */
   private final YoDouble torqueScale;
   private static final double minTorqueScale = 1.0e-2;
   private static final double maxTorqueScale = 1.0e2;

   private final YoDouble gearRatio;
   private final YoDouble kt;

   // Measured data
   private final YoDouble measuredPositionRaw;
   private final YoDouble measuredVelocityRaw;
   private final YoDouble measuredTorqueRaw;

   private final YoDouble measuredPosition;
   private final YoDouble measuredVelocity;
   private final YoDouble measuredTorque;
   private final YoDouble measuredCurrent;

   private final AlphaFilteredYoVariable measuredTorqueFiltered;
   private final YoDouble measuredVelocityAlpha;
   private final AlphaFilteredYoVariable measuredVelocityFiltered;

   // Desired setpoints
   private final YoInteger desiredPositionRaw;
   private final YoInteger desiredVelocityRaw;
   private final YoInteger desiredTorqueRaw;
   private final YoInteger desiredKpRaw;
   private final YoInteger desiredKdRaw;

   private final YoDouble desiredPosition;
   private final YoDouble desiredVelocity;
   private final YoDouble desiredTorque;
   private final YoDouble desiredKp;
   private final YoDouble desiredKd;

   // Temperature model  (should move to own class when tested)
   private final YoDouble estimatedTemp;

   private TMotorTemperatureModel temperatureModel;
   private final TMotorOverTorqueProcessor overTorqueProcessor;

   private double dt;

   public TMotor(int id, String name, TMotorVersion version, double dt, YoRegistry parentRegistry)
   {
      this(id, name, version, dt, null, false, null, parentRegistry);
   }
   
   public TMotor(int id, String name, TMotorVersion version, double dt, YoDouble offsetIntervalRateLimit, boolean useTorqueProcessor, YoDouble yoTime, YoRegistry parentRegistry)
   {
      this.ID = id;
      this.motorName = name;
      this.dt = dt;
      this.version = version;
      String prefix = motorName + "_";

      TMotorParameters motorParameters = version.getMotorParameters();

      motorCommand = new TMotorCommand(ID, motorParameters);
      motorReply = new TMotorReply(motorParameters);

      registry = new YoRegistry(prefix + name);
      debugRegistry = new YoRegistry(prefix + name + "Debug");
      yoCANMsg = new YoCANMsg(motorName, debugRegistry);

      //parameters
      gearRatio = new YoDouble(prefix + "gearRatio", registry);
      torqueScale = new YoDouble(prefix + "torqueScale", registry);
      kt = new YoDouble(prefix + "kt", registry);
      offsetIntervalRequested = new YoInteger(prefix + "offsetInterval", registry);
      if(offsetIntervalRateLimit == null)
         offsetIntervalRateLimited = new RateLimitedYoVariable(prefix + "offsetIntervalRateLimited", registry, 0.3,  dt);
      else
         offsetIntervalRateLimited = new RateLimitedYoVariable(prefix + "offsetIntervalRateLimited", registry, offsetIntervalRateLimit,  dt);
      motorDirection = new YoInteger(prefix + "motorDirection", registry);

      motorDirection.set(1);
      gearRatio.set(motorParameters.getGearRatio());
      torqueScale.set(1.0);
      kt.set(motorParameters.getKt());
      outputAnglePerInputRevolution = 2.0 * Math.PI / motorParameters.getGearRatio();

      // Desireds setpoints
      desiredPositionRaw = new YoInteger(prefix + "desiredPositionRaw", debugRegistry);
      desiredVelocityRaw = new YoInteger(prefix + "desiredVelocityRaw", debugRegistry);
      desiredTorqueRaw = new YoInteger(prefix + "desiredTorqueRaw", debugRegistry);
      desiredKpRaw = new YoInteger(prefix + "desiredKpRaw", debugRegistry);
      desiredKdRaw = new YoInteger(prefix + "desiredKdRaw", debugRegistry);

      desiredPosition = new YoDouble(prefix + "desiredActuatorPosition", registry);
      desiredVelocity = new YoDouble(prefix + "desiredVelocity", registry);
      desiredTorque = new YoDouble(prefix + "desiredTorque", registry);
      desiredKp = new YoDouble(prefix + "desiredKp", registry);
      desiredKd = new YoDouble(prefix + "desiredKd", registry);

      // Measured data
      measuredPositionRaw = new YoDouble(prefix + "measuredPositionRaw", debugRegistry);
      measuredVelocityRaw = new YoDouble(prefix + "measuredVelocityRaw", debugRegistry);
      measuredTorqueRaw = new YoDouble(prefix + "measuredTorqueRaw", debugRegistry);

      measuredPosition = new YoDouble(prefix + "measuredActuatorPosition", registry);
      measuredVelocity = new YoDouble(prefix + "measuredVelocity", registry);
      measuredTorque = new YoDouble(prefix + "measuredTorque", registry);
      measuredCurrent = new YoDouble(prefix + "measuredCurrent", registry);

      measuredTorqueFiltered = new AlphaFilteredYoVariable(prefix + "measuredTorqueFilt", debugRegistry, 0.9, measuredTorque);
      measuredVelocityAlpha = new YoDouble(prefix + "measuredVelocityAlpha", debugRegistry);
      measuredVelocityFiltered = new AlphaFilteredYoVariable(prefix + "measuredVelocityFilt", registry, measuredVelocityAlpha, measuredVelocity);
      measuredVelocityAlpha.set(0.92);

      estimatedTemp = new YoDouble(prefix + "estimatedTemperature", registry);

      CurrentProvider currentProvider = this::getCurrent;
      temperatureModel = new TMotorTemperatureModel(prefix, motorParameters, currentProvider, debugRegistry);

      if (useTorqueProcessor && yoTime != null)
      {
         overTorqueProcessor = new TMotorOverTorqueProcessor(prefix, yoTime, version.getMotorParameters().getTorqueLimitUpper(), torqueScale, measuredTorque, registry);
      }
      else
      {
         overTorqueProcessor = null;
      }

      if(ENABLE_DEBUG_VARIABLES)
      {
         registry.addChild(debugRegistry);
      }
      parentRegistry.addChild(registry);
   }

   /**
    * Reads the data from the PCAN message received. This method shifts the received position using the
    * offset interval, motor direction, and torque scale
    */
   public void read(TPCANMsg message)
   {
      yoCANMsg.setReceived(message);
      motorReply.parseAndUnpack(message);

      offsetIntervalRateLimited.update(offsetIntervalRequested.getValue());

      measuredPositionRaw.set(motorReply.getMeasuredPositionRaw());
      measuredVelocityRaw.set(motorReply.getMeasuredVelocityRaw());
      measuredTorqueRaw.set(motorReply.getMeasuredTorqueRaw());

      double torqueScale = EuclidCoreTools.clamp(this.torqueScale.getValue(), minTorqueScale, maxTorqueScale);
      measuredPosition.set(motorDirection.getValue() * motorReply.getMeasuredPosition() + offsetIntervalRateLimited.getDoubleValue() * outputAnglePerInputRevolution);
      measuredVelocity.set(motorDirection.getValue() * motorReply.getMeasuredVelocity());
      measuredTorque.set(motorDirection.getValue() * motorReply.getMeasuredTorque() * torqueScale);
      measuredCurrent.set(measuredTorque.getDoubleValue() / gearRatio.getDoubleValue() / kt.getDoubleValue());

      if (overTorqueProcessor != null)
      {
         measuredTorque.set(overTorqueProcessor.computeWrapAroundCompensatedTorque());
      }

      measuredVelocityFiltered.update();
      measuredTorqueFiltered.update();

      temperatureModel.update(dt);
      estimatedTemp.set(temperatureModel.getTemperature());
   }

   /**
    * Sets the CAN message to the setpoints provided Overrides any previously set command
    */
   public void setCommand(double kp, double kd, double desiredPosition, double desiredVelocity, double desiredTorque)
   {
      double torqueScale = EuclidCoreTools.clamp(this.torqueScale.getValue(), minTorqueScale, maxTorqueScale);
      double adjustedDesiredPosition = motorDirection.getValue() * (desiredPosition - offsetIntervalRateLimited.getDoubleValue() * outputAnglePerInputRevolution);
      double adjustedDesiredVelocity = motorDirection.getValue() * desiredVelocity;
      double adjustedDesiredTorque = motorDirection.getValue() * desiredTorque / torqueScale;

      this.desiredPosition.set(adjustedDesiredPosition);
      this.desiredVelocity.set(adjustedDesiredVelocity);
      this.desiredTorque.set(adjustedDesiredTorque);
      this.desiredKp.set(kp);
      this.desiredKd.set(kd);

      motorCommand.setCommand(adjustedDesiredPosition, adjustedDesiredVelocity, adjustedDesiredTorque, kp, kd);
      yoCANMsg.setSent(motorCommand.getCANMsg());

      desiredPositionRaw.set(motorCommand.getDesiredPositionRaw());
      desiredVelocityRaw.set(motorCommand.getDesiredVelocityRaw());
      desiredTorqueRaw.set(motorCommand.getDesiredTorqueRaw());
      desiredKpRaw.set(motorCommand.getKpRaw());
      desiredKdRaw.set(motorCommand.getKdRaw());
   }

   /**
    * Sets the CAN message to disable the motor, Overrides any previously set command
    */
   public void setCommandToDisableMotor()
   {
      motorCommand.setCommandToDisableMotor();
      yoCANMsg.setSent(motorCommand.getCANMsg());
   }

   /**
    * Sets the CAN message to enable the motor Overrides any previously set command
    */
   public void setCommandToEnableMotor()
   {
      motorCommand.setCommandToEnableMotor();
      yoCANMsg.setSent(motorCommand.getCANMsg());
   }

   /**
    * Sets the CAN message to zero the encoder at the current location Overrides any previously set
    * command
    */
   public void setCommandToZeroEncoder()
   {
      motorCommand.setCommandToZeroMotor();
      yoCANMsg.setSent(motorCommand.getCANMsg());
   }

   public void setPositionJointOffset(double jointPositionOffset)
   { // initialize measured joint position to be at 0.0
      measuredPosition.set(-jointPositionOffset);
   }

   public String getMotorName()
   {
      return motorName;
   }

   public void reversePositiveMotorDirection()
   {
      motorDirection.set(-1);
   }

   public double getPosition()
   {
      return measuredPosition.getDoubleValue();
   }

   public double getVelocity()
   {
      return measuredVelocity.getDoubleValue();
   }

   public double getFilteredVelocity()
   {
      return measuredVelocityFiltered.getValue();
   }

   public double getTorque()
   {
      return measuredTorque.getDoubleValue();
   }

   public double getFilteredTorque()
   {
      return measuredTorqueFiltered.getDoubleValue();
   }

   public double getEstimatedTemp()
   {
      return estimatedTemp.getDoubleValue();
   }

   public int getDirection()
   {
      return motorDirection.getIntegerValue();
   }

   public void setTorqueScale(double torqueScaling)
   {
      this.torqueScale.set(torqueScaling);
   }

   public void setOffsetInterval(int offsetIntervalRequested)
   {
      this.offsetIntervalRequested.set(offsetIntervalRequested);
   }

   public int getOffsetInterval()
   {
      return offsetIntervalRequested.getValue();
   }

   public int getID()
   {
      return ID;
   }

   public double getOutputAnglePerInputRevolution()
   {
      return outputAnglePerInputRevolution;
   }

   /**
    * After calling set, the returned message is ready to be sent over a PCAN Channel to a TMotor
    * 
    * @return
    */
   public TPCANMsg getCommandedMsg()
   {
      return motorCommand.getCANMsg();
   }

   public double getCurrent()
   {
      return this.measuredCurrent.getDoubleValue();
   }

   public YoInteger getOffsetIntervalRequested()
   {
      return offsetIntervalRequested;
   }

   public void setOverTorqueCompensationEnabled(BooleanSupplier enabled)
   {
      if (overTorqueProcessor != null)
      {
         overTorqueProcessor.setEnabled(enabled);
      }
   }
}