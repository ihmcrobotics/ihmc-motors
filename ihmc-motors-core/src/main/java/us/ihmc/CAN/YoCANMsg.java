package us.ihmc.CAN;

import org.apache.commons.collections.map.CaseInsensitiveMap;
import peak.can.basic.TPCANMsg;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoCANMsg
{
    private final YoRegistry registry;

    private final YoInteger CANReceivedMessageID;
    private final YoInteger CANReceivedMessageBuffer0;
    private final YoInteger CANReceivedMessageBuffer1;
    private final YoInteger CANReceivedMessageBuffer2;
    private final YoInteger CANReceivedMessageBuffer3;
    private final YoInteger CANReceivedMessageBuffer4;
    private final YoInteger CANReceivedMessageBuffer5;
    private final YoInteger CANReceivedMessageBuffer6;
    private final YoInteger CANReceivedMessageBuffer7;
    private final YoInteger[] receivedDataList;

    private final YoInteger CANSentMessageID;
    private final YoInteger CANSentMessageBuffer0;
    private final YoInteger CANSentMessageBuffer1;
    private final YoInteger CANSentMessageBuffer2;
    private final YoInteger CANSentMessageBuffer3;
    private final YoInteger CANSentMessageBuffer4;
    private final YoInteger CANSentMessageBuffer5;
    private final YoInteger CANSentMessageBuffer6;
    private final YoInteger CANSentMessageBuffer7;
    private final YoInteger[] sentDataList;

    public YoCANMsg(String prefix, YoRegistry parentRegistry)
    {
       registry = new YoRegistry(prefix + getClass().getSimpleName());

       CANReceivedMessageID = new YoInteger(prefix + "CANReceivedID", registry);
       CANReceivedMessageBuffer0 = new YoInteger(prefix + "CANReceivedMessageBuffer0", registry); 
       CANReceivedMessageBuffer1 = new YoInteger(prefix + "CANReceivedMessageBuffer1", registry); 
       CANReceivedMessageBuffer2 = new YoInteger(prefix + "CANReceivedMessageBuffer2", registry); 
       CANReceivedMessageBuffer3 = new YoInteger(prefix + "CANReceivedMessageBuffer3", registry); 
       CANReceivedMessageBuffer4 = new YoInteger(prefix + "CANReceivedMessageBuffer4", registry); 
       CANReceivedMessageBuffer5 = new YoInteger(prefix + "CANReceivedMessageBuffer5", registry);
       CANReceivedMessageBuffer6 = new YoInteger(prefix + "CANReceivedMessageBuffer6", registry);
       CANReceivedMessageBuffer7 = new YoInteger(prefix + "CANReceivedMessageBuffer7", registry);

       receivedDataList = new YoInteger[] {CANReceivedMessageBuffer0, CANReceivedMessageBuffer1, CANReceivedMessageBuffer2,
                                           CANReceivedMessageBuffer3, CANReceivedMessageBuffer4, CANReceivedMessageBuffer5,
                                           CANReceivedMessageBuffer6, CANReceivedMessageBuffer7};

       CANSentMessageID = new YoInteger(prefix + "CANSentID", registry);
       CANSentMessageBuffer0 = new YoInteger(prefix + "CANSentMessageBuffer0", registry);         
       CANSentMessageBuffer1 = new YoInteger(prefix + "CANSentMessageBuffer1", registry);         
       CANSentMessageBuffer2 = new YoInteger(prefix + "CANSentMessageBuffer2", registry);         
       CANSentMessageBuffer3 = new YoInteger(prefix + "CANSentMessageBuffer3", registry);         
       CANSentMessageBuffer4 = new YoInteger(prefix + "CANSentMessageBuffer4", registry);         
       CANSentMessageBuffer5 = new YoInteger(prefix + "CANSentMessageBuffer5", registry);         
       CANSentMessageBuffer6 = new YoInteger(prefix + "CANSentMessageBuffer6", registry);         
       CANSentMessageBuffer7 = new YoInteger(prefix + "CANSentMessageBuffer7", registry);

       sentDataList = new YoInteger[] {CANSentMessageBuffer0, CANSentMessageBuffer1, CANSentMessageBuffer2,
                                       CANSentMessageBuffer3, CANSentMessageBuffer4, CANSentMessageBuffer5,
                                       CANSentMessageBuffer6, CANSentMessageBuffer7};
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
}
