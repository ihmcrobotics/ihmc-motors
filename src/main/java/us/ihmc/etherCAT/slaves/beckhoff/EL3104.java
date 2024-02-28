package us.ihmc.etherCAT.slaves.beckhoff;

import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;

public class EL3104 extends Slave
{
   static final int vendorID = 0x00000002;
   static final int productCode = 0x0c203052;
   static final double resolution = Math.pow(2, 16) / 2.0; //resolution of 2^16 across -10v to 10v or +/- 32768
   static final double referenceVoltage = 10.0;
   private final Input in1 = new Input(0x1a00);

   ;
   private final Input in2 = new Input(0x1a01);
   private final Input in3 = new Input(0x1a02);
   private final Input in4 = new Input(0x1a03);
   private final Input[] inputs = {in1, in2, in3, in4};
   public EL3104(int aliasAddress, int configAddress)
   {
      super(vendorID, productCode, aliasAddress, configAddress);

      registerSyncManager(new SyncManager(2, false));
      registerSyncManager(new SyncManager(3, false));

      sm(3).registerPDO(in1);
      sm(3).registerPDO(in2);
      sm(3).registerPDO(in3);
      sm(3).registerPDO(in4);
   }

   public int getIn1()
   {
      return in1.value.get();
   }

   public double getProcessedIn1()
   {
      return (in1.value.get() / resolution) * referenceVoltage;
   }

   public int getIn2()
   {
      return in2.value.get();
   }

   public int getIn3()
   {
      return in3.value.get();
   }

   public int getIn4()
   {
      return in4.value.get();
   }

   public boolean getIn1Error()
   {
      return in1.error.get();
   }

   public boolean getIn1UnderError()
   {
      return in1.underrange.get();
   }

   public boolean getInOver1Error()
   {
      return in1.overrange.get();
   }

   protected Input getInput(int index)
   {
      return inputs[index];
   }

   public class Input extends TxPDO
   {
      Bool underrange = new Bool();
      Bool overrange = new Bool();
      Bit2 limit1 = new Bit2();
      Bit2 limit2 = new Bit2();
      Bool error = new Bool();
      Bool gap1 = new Bool();
      Bit5 gap2 = new Bit5();
      Bool gap3 = new Bool();
      Bool state = new Bool();
      Bool toggle = new Bool();
      Signed16 value = new Signed16();
      protected Input(int address)
      {
         super(address);
      }
   }
}