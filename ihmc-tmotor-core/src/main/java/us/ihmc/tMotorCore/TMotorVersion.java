package us.ihmc.tMotorCore;

import us.ihmc.tMotorCore.parameters.TMotorAK109Parameters;
import us.ihmc.tMotorCore.parameters.TMotorAK109V2Parameters;
import us.ihmc.tMotorCore.parameters.TMotorAK606Parameters;
import us.ihmc.tMotorCore.parameters.TMotorAK809Parameters;
import us.ihmc.tMotorCore.parameters.TMotorParameters;

public enum TMotorVersion
{
   AK606(new TMotorAK606Parameters()), AK809(new TMotorAK809Parameters()), AK109(new TMotorAK109Parameters()), AK109v2(new TMotorAK109V2Parameters());

   private final TMotorParameters tMotorParameters;

   private TMotorVersion(TMotorParameters tMotorParameters)
   {
      this.tMotorParameters = tMotorParameters;
   }

   public TMotorParameters getMotorParameters()
   {
      return tMotorParameters;
   }
}
