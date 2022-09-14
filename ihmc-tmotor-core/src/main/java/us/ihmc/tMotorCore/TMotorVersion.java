package us.ihmc.tMotorCore;

import us.ihmc.tMotorCore.parameters.TMotorAK109Parameters;
import us.ihmc.tMotorCore.parameters.TMotorAK109V2Parameters;
import us.ihmc.tMotorCore.parameters.TMotorAK606Parameters;
import us.ihmc.tMotorCore.parameters.TMotorAK809Parameters;
import us.ihmc.tMotorCore.parameters.TMotorParameters;

public enum TMotorVersion
{
    AK606,
    AK809,
    AK109,
    AK109v2;

    private TMotorParameters tMotorParameters;

    public TMotorParameters getMotorParameters()
    {
        switch (this)
        {
            case AK606:
                tMotorParameters = new TMotorAK606Parameters();
                return tMotorParameters;
            case AK809:
                tMotorParameters = new TMotorAK809Parameters();
                return tMotorParameters;
            case AK109:
                tMotorParameters = new TMotorAK109Parameters();
                return tMotorParameters;
            case AK109v2:
               tMotorParameters = new TMotorAK109V2Parameters();
               return tMotorParameters;                
            default:
                throw new RuntimeException("T-Motor version: Parameters do not exist");

        }
    }
}
