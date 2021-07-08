package us.ihmc.tMotorCore.parameters;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.eva.model.EvaExoskeletonVersion;
import us.ihmc.eva.model.robotVersions.RightHipAndKneeTestStandJointMap;

public class EvaMotorIDParameters
{
    private TObjectIntMap<RightHipAndKneeTestStandJointMap.Joints> jointMotorID = new TObjectIntHashMap<>();

    public EvaMotorIDParameters(EvaExoskeletonVersion version)
    {
        setMotorIDs();
    }

    private void setMotorIDs()
    {
        jointMotorID.put(RightHipAndKneeTestStandJointMap.Joints.r_leg_hpy, 2);
//        jointMotorID.put(RightHipAndKneeTestStandJointMap.Joints.r_leg_kn_pulley_y, 1);
    }

    public int getJointMotorID(RightHipAndKneeTestStandJointMap.Joints joint)
    {
        return jointMotorID.get(joint);
    }

    public boolean isMotorPresent(RightHipAndKneeTestStandJointMap.Joints joint)
    {
        return jointMotorID.containsKey(joint);
    }

    public boolean isAttachedToPulley(RightHipAndKneeTestStandJointMap.Joints joint)
    {
        return (joint.equals(RightHipAndKneeTestStandJointMap.Joints.r_leg_kn_pulley_y));
    }

    public int[] getAllJointMotorIDs()
    {
        return jointMotorID.values();
    }

    public int getZAxisSign(RightHipAndKneeTestStandJointMap.Joints joint)
    {
        switch (joint)
        {
            case r_leg_hpy:
                return 1;
            case r_leg_kn_pulley_y:
                return -1;
            default:
                throw new RuntimeException("Joint name does not have a sign convention");
        }
    }
}
