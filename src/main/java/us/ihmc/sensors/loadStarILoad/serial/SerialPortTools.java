package us.ihmc.sensors.loadStarILoad.serial;

import jssc.SerialPort;
import jssc.SerialPortException;
import jssc.SerialPortList;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public class SerialPortTools
{
   private static SerialPortList serialPortList;

   @SuppressWarnings("unchecked")
   public static void printSerialPortNames()
   {
      serialPortList = new SerialPortList();
      String[] portList = serialPortList.getPortNames();

      System.out.println("Serial ports:");

      for (String port : portList)
      {
         System.out.println(port);
      }
   }

   public static SerialPort openSerialPort(String portName,
                                           String className,
                                           int baudRate,
                                           int dataBits,
                                           int stopBits,
                                           int parity,
                                           int flowControlMode,
                                           int timeOut) throws IOException
   {
      SerialPort serialPort = null;

      try
      {
         serialPort = new SerialPort(portName);
         serialPort.openPort();
         serialPort.setParams(baudRate, dataBits, stopBits, parity);
         serialPort.setFlowControlMode(flowControlMode);
      }
      catch (Exception e)
      {
         throw new IOException(e.getMessage());
      }

      return serialPort;
   }

   public static void closeSerialPort(SerialPort serialPort) throws SerialPortException
   {
      serialPort.closePort();
   }

   public static void sendByte(int b1, OutputStream outputStream)
   {
      sendByteArray(new int[] {b1}, outputStream);
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

   public static void sendByteArray(int[] cmd, SerialPort serialPort)
   {
      try
      {
         for (int outByte : cmd)
         {
            serialPort.writeInt(outByte);
         }
      }
      catch (SerialPortException e)
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
