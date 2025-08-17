package pedroPathing.sensors;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

/**
 * Functional VL53L5CX I2C driver for FTC
 */
public class VL53L5CXGrid extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private static final int BYTES_PER_FRAME = 1 + 1 + 64 * 2; // status + frame + 64 cells
    private static final int REGISTER_START = 0x00;

    // Constructor
    public VL53L5CXGrid(I2cDeviceSynch deviceClient, int i2c7bitAddress) {
        super(deviceClient, true); // true = auto-update
        deviceClient.setI2cAddress(I2cAddr.create7bit(i2c7bitAddress));
        deviceClient.engage();

        // Read all bytes automatically
        deviceClient.setReadWindow(
                new I2cDeviceSynch.ReadWindow(
                        REGISTER_START, BYTES_PER_FRAME, I2cDeviceSynch.ReadMode.REPEAT));
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

    /** Frame class: stores one sensor reading */
    public static class Frame {
        public int status;
        public int frameCounter;
        public int[][] mm = new int[8][8];
    }

    /** Read one full frame from the sensor */
    public Frame readFrame() {
        byte[] buf = deviceClient.read(REGISTER_START, BYTES_PER_FRAME);
        Frame f = new Frame();
        f.status = buf[0] & 0xFF;
        f.frameCounter = buf[1] & 0xFF;

        int idx = 2;
        for (int r = 0; r < 8; r++) {
            for (int c = 0; c < 8; c++) {
                int lo = buf[idx++] & 0xFF;
                int hi = buf[idx++] & 0xFF;
                f.mm[r][c] = (hi << 8) | lo; // little-endian
            }
        }
        return f;
    }

    /** Helper: find nearest cell in a frame */
    public int[] getNearestCell(Frame f) {
        int best = Integer.MAX_VALUE;
        int bestR = -1, bestC = -1;
        for (int r = 0; r < 8; r++) {
            for (int c = 0; c < 8; c++) {
                int d = f.mm[r][c];
                if (d > 0 && d < best) {
                    best = d;
                    bestR = r;
                    bestC = c;
                }
            }
        }
        return new int[]{bestR, bestC, best};
    }
}
