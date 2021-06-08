package us.ihmc.sensors.LoadStarILoad.serial;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Enumeration;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.NoSuchPortException;
import gnu.io.SerialPort;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;


public class SerialPortTools
{
   @SuppressWarnings("unchecked")
   public static void printSerialPortNames()
   {
      Enumeration<CommPortIdentifier> portEnumeration;
      ArrayList<String> portList = new ArrayList<String>();
      portEnumeration = CommPortIdentifier.getPortIdentifiers();

      CommPortIdentifier portId;
      while (portEnumeration.hasMoreElements())
      {
         portId = portEnumeration.nextElement();

         if (portId.getPortType() == CommPortIdentifier.PORT_SERIAL)
         {
            portList.add(portId.getName());
         }
      }

      System.out.println("Serial ports:");

      for (String port : portList)
      {
         System.out.println(port);
      }
   }
   
   public static SerialPort openSerialPort(String portName, String className, int baudRate, int dataBits, int stopBits, int parity, int flowControlMode, int timeOut)
           throws IOException
   {
      SerialPort serialPort = null;

      try
      {
         CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
         if (portIdentifier.isCurrentlyOwned())
         {
             System.err.println("Error: Port is currently in use");
         }
         CommPort commPort = portIdentifier.open(className, timeOut);
         if (!(commPort instanceof SerialPort))
         {
            throw new NoSuchPortException();
         }

         serialPort = (SerialPort) commPort;

         serialPort.setSerialPortParams(baudRate, dataBits, stopBits, parity);
         serialPort.setFlowControlMode(flowControlMode);
      }
      catch (Exception e)
      {
         throw new IOException(e.getMessage());
      }

      return serialPort;
   }
   
   public static void closeSerialPort(SerialPort serialPort)
   {
      serialPort.close();
   }

   public static void sendByte(int b1, OutputStream outputStream)
   {
      sendByteArray(new int[] { b1 }, outputStream);
   }

   public static void sendByteArray(int[] cmd, OutputStream outputStream)
   {
      try
      {
         for (int outByte : cmd)
         {
            outputStream.write(outByte);
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   public static int readByteArray(int[] result, InputStream inputStream, int off, int len) throws IOException
   {
      for (int i = off; i < off + len; i++)
      {
         int res = inputStream.read();
         if (res == -1)
            return i - off;
         result[i] = res;
      }
      return len;
   }
   
   public static void createReaderThread(SerialPortReader serialPortReader, String threadName)
   {
      Thread serialPortReaderThread = new Thread(serialPortReader, threadName);
      serialPortReaderThread.start();
   }
   
   public static void createReaderThread(SerialPortReader serialPortReader)
   {
	   Thread serialPortReaderThread = new Thread(serialPortReader);
	   serialPortReaderThread.start();
   }

   public static void createReaderThread(SerialPortReader serialPortReader, PriorityParameters priorityParameters)
   {
      RealtimeThread serialPortReaderThread = new RealtimeThread(priorityParameters, serialPortReader);
      serialPortReaderThread.start();
   }
}
