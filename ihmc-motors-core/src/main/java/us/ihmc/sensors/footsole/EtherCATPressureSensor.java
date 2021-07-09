package us.ihmc.sensors.footsole;

import us.ihmc.etherCAT.javalution.Struct;
import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;


public class EtherCATPressureSensor extends Slave
{
    static final int vendorID = 0x603;
    static final int productCode = 0x202;

    public class RPDO_1600 extends RxPDO
    {
        protected RPDO_1600(int address)
        {
            super(address);
        }

        Signed16 reset = new Signed16();
    }

    public class TPDO_1AAA extends TxPDO
    {
        protected TPDO_1AAA(int address)
        {
            super(address);
        }

        Signed16 pressureMbar0 = new Signed16();
        Signed16 pressureMbar1 = new Signed16();
        Signed16 pressureMbar2 = new Signed16();
        Signed16 pressureMbar3 = new Signed16();
        Signed16 pressureMbar4 = new Signed16();
        Signed16 pressureMbar5 = new Signed16();
        Signed16 pressureMbar6 = new Signed16();
        Signed16 pressureMbar7 = new Signed16();
        Signed16 duration = new Signed16();
        Signed16 cpt = new Signed16();
    }

    private final RPDO_1600 rpdo_x1600;
    private final TPDO_1AAA tpdo_x1A00;

    public EtherCATPressureSensor(int alias, int ringPosition)
    {
        super(vendorID, productCode, alias, ringPosition);

        rpdo_x1600 = new RPDO_1600(0x1600);
        tpdo_x1A00 = new TPDO_1AAA(0x1AAA);

        registerSyncManager(new SyncManager(2, false));
        registerSyncManager(new SyncManager(3,false));

        sm(2).registerPDO(rpdo_x1600);
        sm(3).registerPDO(tpdo_x1A00);
    }

    public int getPressureMbar(int index)
    {
        switch (index)
        {
            case 0:
                return tpdo_x1A00.pressureMbar0.get();
            case 1:
                return tpdo_x1A00.pressureMbar1.get();
            case 2:
                return tpdo_x1A00.pressureMbar2.get();
            case 3:
                return tpdo_x1A00.pressureMbar3.get();
            case 4:
                return tpdo_x1A00.pressureMbar4.get();
            case 5:
                return tpdo_x1A00.pressureMbar5.get();
            case 6:
                return tpdo_x1A00.pressureMbar6.get();
            case 7:
                return tpdo_x1A00.pressureMbar7.get();
            default:
                return 0;
        }
    }

    public Struct.Signed16 getDuration()
    {
        return tpdo_x1A00.duration;
    }

    public Struct.Signed16 getFrequency()
    {
        return tpdo_x1A00.cpt;
    }


}
