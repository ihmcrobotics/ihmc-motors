package us.ihmc.sensors.LoadStarILoad.serial;

import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import jssc.SerialPort;
import jssc.SerialPortException;
import jssc.SerialPortList;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.List;

public class Serial
{
    /* List of usual serial ports.
     * Note: Linux might need permissions to connect: sudo chown username /dev/ttyACM0 */
    private static final List<String> USUAL_PORTS = Arrays.asList("/dev/tty.ustringBuildermodem", "/dev/tty.ustringBuilderserial", // Mac OS X
            "/dev/ustringBuilderdev", "/dev/ttyUstringBuilder", "/dev/ttyACM", "/dev/serial", // Linux
            "COM3", "COM4", "COM5", "COM6", "COM9" // Windows
    );

    private final int ARDUINO_SERIAL_BAUDRATE = 9600; //921600; //115200
    private final String arduinoPort;
    private SerialPort serialPort;
    public static final String SEPARATOR = ";";
    private StringBuilder stringBuilder = new StringBuilder();
    private final StringProperty line = new SimpleStringProperty("");

    public Serial()
    {
        arduinoPort = "";
    }

    public Serial(String port)
    {
        arduinoPort = port;
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
                        ((!arduinoPort.isEmpty() && name.equals(arduinoPort)) ||
                                (arduinoPort.isEmpty() &&
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
                            serialPort.setParams(ARDUINO_SERIAL_BAUDRATE,
                                    SerialPort.DATABITS_8,
                                    SerialPort.STOPBITS_1,
                                    SerialPort.PARITY_NONE);
                        }
                        serialPort.setEventsMask(SerialPort.MASK_RXCHAR);
                        serialPort.addEventListener(event -> {
                            if (event.isRXCHAR())
                            {
                                try
                                {
                                    stringBuilder.append(serialPort.readString(event.getEventValue()));
                                    String ch = stringBuilder.toString();
                                    if (ch.endsWith("\r\n"))
                                    {
                                        // add timestamp
                                        line.set(Long.toString(System.currentTimeMillis()).
                                                concat(SEPARATOR).
                                                concat(ch.substring(0, ch.indexOf("\r\n"))));
                                        stringBuilder = new StringBuilder();
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

    public void connect(SerialPort serialPort) throws SerialPortException {
        this.serialPort = serialPort;

        serialPort.setEventsMask(SerialPort.MASK_RXCHAR);
        serialPort.addEventListener(event -> {
            if (event.isRXCHAR())
            {
                try
                {
                    stringBuilder.append(serialPort.readString(event.getEventValue()));
                    String ch = stringBuilder.toString();
                    if (ch.endsWith("\r\n"))
                    {
                        // add timestamp
                        line.set(Long.toString(System.currentTimeMillis()).
                                concat(SEPARATOR).
                                concat(ch.substring(0, ch.indexOf("\r\n"))));
                        stringBuilder = new StringBuilder();
                    }
                }
                catch (SerialPortException e)
                {
                    System.out.println("SerialEvent error:" + e.toString());
                }
            }
        });
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

    public StringProperty getLine()
    {
        return line;
    }

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
}