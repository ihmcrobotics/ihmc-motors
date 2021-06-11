package us.ihmc.sensors.LoadStarILoad.serial;

import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import jssc.SerialPort;
import jssc.SerialPortException;
import jssc.SerialPortList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.sensors.LoadStarILoad.LoadStarILoadCallback;
import us.ihmc.sensors.LoadStarILoad.LoadStarILoadParser;
import us.ihmc.sensors.LoadStarILoad.LoadStarILoadWriter;
import us.ihmc.sensors.LoadStarILoad.settings.LoadStarILoadCommandEnum;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.List;

public class SerialLoadcell
{
    /* List of usual serial ports.
     * Note: Linux might need permissions to connect: sudo chown username /dev/ttyACM0 */
    private static final List<String> USUAL_PORTS = Arrays.asList("/dev/tty.ustringBuildermodem", "/dev/tty.ustringBuilderserial", // Mac OS X
            "/dev/ustringBuilderdev", "/dev/ttyUstringBuilder", "/dev/ttyACM", "/dev/serial", // Linux
            "COM3", "COM4", "COM5", "COM6", "COM9" // Windows
    );

    private final int SERIAL_BAUDRATE = 9600; //921600; //115200
    private final String COMPort;
    private SerialPort serialPort;
    public static final String SEPARATOR = ";";
    private StringBuilder stringBuilder = new StringBuilder();
//    private final StringProperty line = new SimpleStringProperty("");
    private double force;

    private LoadStarILoadParser parser;
    private LoadStarILoadCallback loadStarILoadCallback;

    public SerialLoadcell()
    {
        COMPort = "";
    }

    public SerialLoadcell(String port)
    {
        COMPort = port;
    }

    /* connect() looks for a valid serial port with an Arduino board connected.
     * If it is found, it's opened and a listener is added, so every time
     * a line is returned, the stringProperty is set with that line.
     * For that, a StringBuilder is used to store the chars and extract the line
     * content whenever '\r\n' is found.    */
    public boolean connect()
    {
        Arrays.asList(SerialPortList.getPortNames()).stream()
                .filter(name ->
                        ((!COMPort.isEmpty() && name.equals(COMPort)) ||
                                (COMPort.isEmpty() &&
                                        USUAL_PORTS.stream().
                                                anyMatch(p -> name.startsWith(p))))).
                findFirst().
                ifPresent(name ->
                {
                    try
                    {
                        serialPort = new SerialPort(name);
                        System.out.println("Connecting to " + serialPort.getPortName());
                        if (serialPort.openPort())
                        {
                            serialPort.setParams(SERIAL_BAUDRATE,
                                    SerialPort.DATABITS_8,
                                    SerialPort.STOPBITS_1,
                                    SerialPort.PARITY_NONE);
                            serialPort.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);
                        }

                        LogTools.info("Connected: " + serialPort.getPortName());

                        loadStarILoadCallback = new LoadStarILoadCallback();
                        parser = new LoadStarILoadParser(loadStarILoadCallback);

                        // check initial reading //
                        outputWeightOnce();
                        System.out.println("Initial reading: " + serialPort.readString());

                        System.out.println("Creating event listener for serial port");
                        serialPort.setEventsMask('A');
                        serialPort.addEventListener(event -> {
                            if (event.isRXCHAR())
                            {
                                try
                                {
                                    int [] byteArray = serialPort.readIntArray();
                                    if (byteArray != null)
                                    {
                                        for(int byteValue : byteArray)
                                            parser.parseByte(byteValue);
                                        force = loadStarILoadCallback.getForceNewton();
                                    }
                                }
                                catch (SerialPortException e)
                                {
                                    System.out.println("SerialEvent error:" + e.toString());
                                }
                            }
                        });
                    }
                    catch (SerialPortException ex)
                    {
                        System.out.println("ERROR: Port '" + name + "': " + ex.toString());
                    }
                });
        return serialPort != null;
    }

    public void outputWeightOnce()
    {
        parser.expectCommand(null);
        sendCommand(LoadStarILoadCommandEnum.PING); // stop continuous stuff
        ThreadTools.sleep(100L); // wait until done receiving data
        parser.expectCommand(LoadStarILoadCommandEnum.OUTPUT_WEIGHT_ONCE);
        sendCommand(LoadStarILoadCommandEnum.OUTPUT_WEIGHT_ONCE);
        ThreadTools.sleep(100L); // don't allow any new commands until the last one is processed.
    }

    public void ping()
    {
        parser.expectCommand(null);
        sendCommand(LoadStarILoadCommandEnum.PING); // stop continuous stuff
    }

    public void requestWeight()
    {
        parser.expectCommand(LoadStarILoadCommandEnum.OUTPUT_WEIGHT_ONCE);
        sendCommand(LoadStarILoadCommandEnum.OUTPUT_WEIGHT_ONCE);
    }

    public String readSerial() throws SerialPortException {
        return serialPort.readString();
    }

    public double readForce()
    {
        return force;
    }

    private void sendCommand(LoadStarILoadCommandEnum commandEnum)
    {
        write(commandEnum.getCommand());
    }

    public void disconnect()
    {
        if (serialPort != null)
        {
            try
            {
                serialPort.removeEventListener();
                if (serialPort.isOpened())
                {
                    serialPort.closePort();
                }
            }
            catch (SerialPortException ex)
            {
                System.out.println("ERROR closing port exception: " + ex.toString());
            }
            System.out.println("Disconnecting: comm port closed.");
        }
    }

//    public StringProperty getLine()
//    {
//        return line;
//    }

    public String getPortName()
    {
        return serialPort != null ? serialPort.getPortName() : "";
    }

    public void write(String text)
    {
        try
        {
            serialPort.writeBytes(text.getBytes());
        }
        catch (SerialPortException ex)
        {
            System.out.println("ERROR: writing '" + text + "': " + ex.toString());
        }
    }

    public void write(float value)
    {
        try
        {
            serialPort.writeBytes(ByteBuffer.allocate(4).putFloat(value).array());
        }
        catch (SerialPortException ex)
        {
            System.out.println("ERROR: writing '" + value + " in byte array': " + ex.toString());
        }
    }

    public void write(int[] value)
    {
        try
        {
            for (int outByte : value)
            {
                serialPort.writeInt(outByte);
            }
        }
        catch (SerialPortException ex)
        {
            System.out.println("ERROR: writing '" + value + " in byte array': " + ex.toString());
        }
    }
}