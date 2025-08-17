package pedroPathing.drivers;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

/**
 * Generic I2C sensor driver template for FTC SDK
 */
public class GenericI2CSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private static final int DEFAULT_REGISTER_START = 0x00;
    private static final int DEFAULT_FRAME_SIZE = 2; // change based on your sensor

    /** Constructor using an I2C device client */
    public GenericI2CSensor(I2cDeviceSynch deviceClient) {
        super(deviceClient, true); // true = auto-update
        deviceClient.engage();
    }

    /** Optional: set I2C address if your sensor is not default */
    public void setI2cAddress(int addr7bit) {
        deviceClient.setI2cAddress(I2cAddr.create7bit(addr7bit));
    }

    /** Read a single byte from a register */
    public int readRegister(int reg) {
        return deviceClient.read8(reg) & 0xFF;
    }

    /** Write a single byte to a register */
    public void writeRegister(int reg, byte value) {
        deviceClient.write8(reg, value);
    }

    /** Read multiple bytes from the sensor */
    public byte[] readRegisters(int start, int length) {
        return deviceClient.read(start, length);
    }

    @Override
    protected boolean doInitialize() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return "";
    }

    /** Frame class: wrap sensor data into a structured object */
    public static class Frame {
        public int value1;
        public int value2;
        // Add more fields depending on your sensor
    }

    /** Read a “frame” from the sensor */
    public Frame readFrame() {
        byte[] buf = readRegisters(DEFAULT_REGISTER_START, DEFAULT_FRAME_SIZE);
        Frame f = new Frame();
        f.value1 = buf[0] & 0xFF;
        f.value2 = buf[1] & 0xFF;
        return f;
    }

    /** Example helper: get nearest value from a frame (if your sensor has multiple distances) */
    public int getNearest(Frame f) {
        // Replace with your logic; for 2-value example:
        return Math.min(f.value1, f.value2);
    }
}
