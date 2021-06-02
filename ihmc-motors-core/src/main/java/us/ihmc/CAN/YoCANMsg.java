package us.ihmc.CAN;

import peak.can.basic.TPCANMsg;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoCANMsg
{
    private final YoRegistry registry;

    private final YoInteger CANReceivedMessageBuffer0;
    private final YoInteger CANReceivedMessageBuffer1;
    private final YoInteger CANReceivedMessageBuffer2;
    private final YoInteger CANReceivedMessageBuffer3;
    private final YoInteger CANReceivedMessageBuffer4;
    private final YoInteger CANReceivedMessageBuffer5;

    private final YoInteger CANSentMessageBuffer0;
    private final YoInteger CANSentMessageBuffer1;
    private final YoInteger CANSentMessageBuffer2;
    private final YoInteger CANSentMessageBuffer3;
    private final YoInteger CANSentMessageBuffer4;
    private final YoInteger CANSentMessageBuffer5;
    private final YoInteger CANSentMessageBuffer6;
    private final YoInteger CANSentMessageBuffer7;

    public YoCANMsg(String prefix, int ID, YoRegistry parentRegistry)
    {
       registry = new YoRegistry(prefix + getClass().getSimpleName());
       
       CANReceivedMessageBuffer0 = new YoInteger(prefix + "CANReceivedMessageBuffer0", registry); 
       CANReceivedMessageBuffer1 = new YoInteger(prefix + "CANReceivedMessageBuffer1", registry); 
       CANReceivedMessageBuffer2 = new YoInteger(prefix + "CANReceivedMessageBuffer2", registry); 
       CANReceivedMessageBuffer3 = new YoInteger(prefix + "CANReceivedMessageBuffer3", registry); 
       CANReceivedMessageBuffer4 = new YoInteger(prefix + "CANReceivedMessageBuffer4", registry); 
       CANReceivedMessageBuffer5 = new YoInteger(prefix + "CANReceivedMessageBuffer5", registry); 
                                                                                         
       CANSentMessageBuffer0 = new YoInteger(prefix + "CANSentMessageBuffer0", registry);         
       CANSentMessageBuffer1 = new YoInteger(prefix + "CANSentMessageBuffer1", registry);         
       CANSentMessageBuffer2 = new YoInteger(prefix + "CANSentMessageBuffer2", registry);         
       CANSentMessageBuffer3 = new YoInteger(prefix + "CANSentMessageBuffer3", registry);         
       CANSentMessageBuffer4 = new YoInteger(prefix + "CANSentMessageBuffer4", registry);         
       CANSentMessageBuffer5 = new YoInteger(prefix + "CANSentMessageBuffer5", registry);         
       CANSentMessageBuffer6 = new YoInteger(prefix + "CANSentMessageBuffer6", registry);         
       CANSentMessageBuffer7 = new YoInteger(prefix + "CANSentMessageBuffer7", registry);         
        parentRegistry.addChild(registry);
    }

    public void setReceived(TPCANMsg msg)
    {
        CANReceivedMessageBuffer0.set(msg.getData()[0]);
        CANReceivedMessageBuffer1.set(msg.getData()[1]);
        CANReceivedMessageBuffer2.set(msg.getData()[2]);
        CANReceivedMessageBuffer3.set(msg.getData()[3]);
        CANReceivedMessageBuffer4.set(msg.getData()[4]);
        CANReceivedMessageBuffer5.set(msg.getData()[5]);
    }

    public void setSent(byte[] msg)
    {
        CANSentMessageBuffer0.set(msg[0]);
        CANSentMessageBuffer1.set(msg[1]);
        CANSentMessageBuffer2.set(msg[2]);
        CANSentMessageBuffer3.set(msg[3]);
        CANSentMessageBuffer4.set(msg[4]);
        CANSentMessageBuffer5.set(msg[5]);
        CANSentMessageBuffer6.set(msg[6]);
        CANSentMessageBuffer7.set(msg[7]);
    }
}
