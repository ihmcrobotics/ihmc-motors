package us.ihmc.etherCAT.slaves.beckhoff;

import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class EL3062 extends Slave
{
   private static final int VENDOR_ID = 0x00000002;
   private static final int PRODUCT_CODE = 0x0bf63052;

   protected static final double RESOLUTION = Short.MAX_VALUE; // 12-bit resolution, 16 bit presentation
   protected static final double MAX_VOLTAGE = 10.0;

   public enum AnalogInputChannel
   {
      ONE(0), TWO(1);

      private final int index;

      AnalogInputChannel(int index)
      {
         this.index = index;
      }

      public int getIndex()
      {
         return index;
      }
   }

   class AnalogInput extends TxPDO
   {
      Bool underrange = new Bool();
      Bool overrange = new Bool();
      Bit2 limit1 = new Bit2();
      Bit2 limit2 = new Bit2();
      Bool error = new Bool();
      Bit7 gap = new Bit7();
      Bool txPdoState = new Bool();
      Bool txPdoToggle = new Bool();
      Signed16 value = new Signed16();

      /**
       * Create new TxPDO
       *
       * @param address Address of the PDO
       */
      protected AnalogInput(int address)
      {
         super(address);
      }
   }

   private AnalogInput[] inputs = new AnalogInput[2];

   /**
    * Create a new slave and set the address
    *
    * Slaves are addressed based on their aliasAddress and their position relative to the previous set aliasAddress.
    *
    * The alias can be changed with the "eepromtool" utility that comes with the SOEM master. Power cycle the slave after running.
    *
    * - When the alias is set and unique on the ring, the aliasAddress is the same as the address and the position is 0
    * - When the alias is set and the same as the previous alias on the ring, the aliasAddress is the same as the address and the position is incremented by one.
    * - When the alias is not set or set to zero, the aliasAddress is inherted from the previous slave on the ring and the position is incremented by one.
    * - When the alias is set and the same as a non-adjacent slave on the ring, the slave configuration is considered invalid and the master will not start.
    *
    * @param aliasAddress The address of the slave.
    * @param position Position relative to aliasAddress
    */
   public EL3062(int aliasAddress, int position)
   {
      super(VENDOR_ID, PRODUCT_CODE, aliasAddress, position);

      inputs[0] = new AnalogInput(0x1a00);
      inputs[1] = new AnalogInput(0x1a02);

      registerSyncManager(new SyncManager(3, false));
      sm(3).registerPDO(inputs[0]);
      sm(3).registerPDO(inputs[1]);
   }

   /**
    * Indicates if the voltage is under the measurement range for the device
    * 
    * @see #getError(AnalogInputChannel) 
    * @param channel the channel (one or two)
    * @return True if the measurement is under range
    */
   public boolean getIsUnderrange(AnalogInputChannel channel)
   {
      return inputs[channel.index].underrange.get();
   }

   /**
    * Indicates if the voltage is over the measurement range for the device
    *
    * @see #getError(AnalogInputChannel)
    * @param channel the channel (one or two)
    * @return True if the measurement is over range
    */
   public boolean getIsOverrange(AnalogInputChannel channel)
   {
      return inputs[channel.index].overrange.get();
   }

   /**
    * Returns the status of limit monitor 1
    * 0: not active
    * 1: Value is smaller than limit value 1
    * 2: Value is larger than limit value 1
    * 3: Value is equal to limit value 1
    * @param channel the channel (one or two)
    * @return the limit monitor status
    */
   public short getLimit1Status(AnalogInputChannel channel)
   {
      return inputs[channel.index].limit1.shortValue();
   }

   /**
    * Returns the status of limit monitor 2
    * 0: not active
    * 1: Value is smaller than limit value 2
    * 2: Value is larger than limit value 2
    * 3: Value is equal to limit value 2
    * @param channel the channel (one or two)
    * @return the limit monitor status
    */
   public short getLimit2Status(AnalogInputChannel channel)
   {
      return inputs[channel.index].limit2.shortValue();
   }

   /**
    * Returns true if the channel is in an error state.
    * @see #getIsOverrange(AnalogInputChannel) 
    * @see #getIsUnderrange(AnalogInputChannel)
    * @param channel the channel (one or two)
    * @return true if the measurement is invalid
    */
   public boolean getError(AnalogInputChannel channel)
   {
      return inputs[channel.index].error.get();
   }

   /**
    * Return whether or not the data of the associated TxPDO is <i>INVALID</i> or not
    * @param channel the channel (one or two)
    * @return true if the data is INVALID, false if it is valid
    */
   public boolean getTxPdoState(AnalogInputChannel channel)
   {
      return inputs[channel.index].txPdoState.get();
   }

   /**
    * The TxPDO toggle is toggled by the slave when the data of the associated
    * TxPDO is updated.
    * 
    * @param channel the channel (one or two)
    * @return the bit to monitor for changes
    */
   public boolean getTxPdoToggle(AnalogInputChannel channel)
   {
      return inputs[channel.index].txPdoToggle.get();
   }

   /**
    * Returns the raw value of the analog input measurement. The device itself is capable
    * of 12-bit resolution. Value is stored and reported as signed 16 bit. 0 is 0v, 4096 is +10v
    *
    * @param channel the channel (one or two)
    * @return The measured value
    */
   public short getValueRaw(AnalogInputChannel channel)
   {
      return inputs[channel.index].value.get();
   }

   /**
    * Returns the sensed voltage on the analog input
    * @param channel the channel (one or two)
    * @return the sensed voltage from 0.0 to 10.0v
    */
   public double getValueProcessed(AnalogInputChannel channel)
   {
      return (getValueRaw(channel) / RESOLUTION) * MAX_VOLTAGE;
   }
   
   /**
    * returns the internal input object, used for the YoWrapper
    * @param channel
    * @return
    */
   protected AnalogInput getInput(int channel)
   {
      return inputs[channel];
   }
}
