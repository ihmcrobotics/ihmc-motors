package us.ihmc.tMotorCore;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class TMotorBrownoutManager
{
   private final TMotor tMotor;

   private final TmotorMotorOffDetector offDetector;
   private final TMotorPositionJumpDetector positionJumpDetector;
   private final YoBoolean requestedMotorEnabled;
   private boolean lastValueMotorIsOff;
   private final YoBoolean brownoutOccurred;

   public TMotorBrownoutManager(String prefix, TMotor tMotor, YoBoolean requestedMotorEnabled, YoRegistry registry)
   {
      brownoutOccurred = new YoBoolean(prefix + "BrownoutOccurred", registry);
      brownoutOccurred.set(false);

      this.tMotor = tMotor;
      positionJumpDetector = new TMotorPositionJumpDetector(prefix, new DoubleProvider()
      {
         @Override
         public double getValue()
         {
            return tMotor.getPosition();
         }
      }, TMotor.TICKS_PER_UPDATE, registry);

      offDetector = new TmotorMotorOffDetector(prefix, new DoubleProvider()
      {
         @Override
         public double getValue()
         {
            return tMotor.getTorque();
         }
      }, TMotor.TICKS_PER_UPDATE, registry);

      this.requestedMotorEnabled = requestedMotorEnabled;
   }

   public void update()
   {
      lastValueMotorIsOff = offDetector.getMotorIsOff();
      offDetector.update();
      positionJumpDetector.update();

      // if motor is enabled, and detect transition motor on->motor off, disable
      // TODO need to debug this step
//      if (requestedMotorEnabled.getValue())
//      {
//         if (lastValueMotorIsOff == false && offDetector.getMotorIsOff() == true)
//         {
//            brownoutOccurred.set(true);
//         }
//      }
   }

   public boolean brownoutOccurred()
   {
      return brownoutOccurred.getValue();
   }

   public void reset()
   {
      brownoutOccurred.set(false);
   }
}
