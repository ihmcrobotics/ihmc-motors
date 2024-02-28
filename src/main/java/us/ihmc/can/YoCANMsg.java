package us.ihmc.can;

import peak.can.basic.TPCANMsg;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoCANMsg
{
   private static final int BUFFER_SIZE = 8;
   private final YoRegistry registry;
   private final YoInteger CANReceivedMessageID;
   private final YoInteger CANSentMessageID;

   private final YoInteger[] receivedDataList = new YoInteger[BUFFER_SIZE];
   private final YoInteger[] sentDataList = new YoInteger[BUFFER_SIZE];

   public YoCANMsg(String prefix, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(prefix + getClass().getSimpleName());

      CANReceivedMessageID = new YoInteger(prefix + "CANReceivedID", registry);
      CANSentMessageID = new YoInteger(prefix + "CANSentID", registry);

      for (int i = 0; i < BUFFER_SIZE; i++)
      {
         receivedDataList[i] = new YoInteger(prefix + "CANReceivedMessageBuffer" + i, registry);
         sentDataList[i] = new YoInteger(prefix + "CANSentMessageBuffer" + i, registry);
      }

      parentRegistry.addChild(registry);
   }

   public void setReceived(TPCANMsg msg)
   {
      CANReceivedMessageID.set(msg.getID());
      setReceived(msg.getData(), msg.getLength());
   }

   public void setReceived(byte[] msg, int length)
   {
      for (int i = 0; i < length; i++)
      {
         receivedDataList[i].set(msg[i]);
      }
      for (int i = length; i < 8; i++)
      {
         receivedDataList[i].set(0);
      }
   }

   public void setSent(TPCANMsg msg)
   {
      CANSentMessageID.set(msg.getID());
      setSent(msg.getData(), msg.getLength());
   }

   public void setSent(byte[] msg, int length)
   {
      for (int i = 0; i < length; i++)
      {
         sentDataList[i].set(msg[i]);
      }
      for (int i = length; i < 8; i++)
      {
         sentDataList[i].set(0);
      }
   }

   public boolean receiveMessageIsAllZeros()
   {
      for (int i = 0; i < BUFFER_SIZE; i++)
      {
         if (receivedDataList[i].getValue() != 0)
            return false;
      }

      return true;
   }
}
