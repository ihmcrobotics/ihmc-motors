package us.ihmc.etherCAT.slaves.beckhoff;

import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;

public class EL9510 extends Slave
{

   private final StatusUo statusUo = new StatusUo();

   public EL9510(int aliasAddress, int position)
   {
      super(0x00000002, 0x25263052, aliasAddress, position);
      registerSyncManager(new SyncManager(0, false));
      sm(0).registerPDO(statusUo);
   }

   public class StatusUo extends TxPDO
   {
      Bool powerOk = new Bool();
      Bool overload = new Bool();
      public StatusUo()
      {
         super(0x1A00);
      }
   }
}