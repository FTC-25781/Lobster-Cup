package pedroPathing.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadWindow;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadMode;

public class VL53L5CXGrid {
    private static final int BYTES_PER_FRAME = 1 + 1 + 64 * 2; // status + frame + 64 cells (2 bytes each) = 130
    private static final int REGISTER_START = 0x00;

    private final I2cDeviceSynch device;

    public VL53L5CXGrid(HardwareMap hw, String name, int i2c7bitAddress) {
        // Get device as I2cDeviceSynch (not Impl!)
        device = hw.get(I2cDeviceSynch.class, name);
        device.setI2cAddress(I2cAddr.create7bit(i2c7bitAddress));
        device.engage();

        // Define a read window for the entire frame
        ReadWindow window = new ReadWindow(REGISTER_START, BYTES_PER_FRAME, ReadMode.REPEAT);
        device.setReadWindow(window);
    }

    // Frame structure for one measurement
    public static class Frame {
        public int status;              // 0 = OK
        public int frameCounter;        // rolling counter
        public int[][] mm = new int[8][8]; // millimeter distances
    }

    // Reads one frame from the sensor
    public Frame readFrame() {
        byte[] buf = device.read(REGISTER_START, BYTES_PER_FRAME);
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
}
