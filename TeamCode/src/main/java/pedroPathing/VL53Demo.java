package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import pedroPathing.sensors.VL53L5CXGrid;

@TeleOp(name = "VL53L5CX Demo")
public class VL53Demo extends LinearOpMode {

    private VL53L5CXGrid tof;

    @Override
    public void runOpMode() {

        // Initialize I2C device from HardwareMap
        I2cDeviceSynch device = hardwareMap.get(I2cDeviceSynch.class, "tofBridge");
        tof = new VL53L5CXGrid(device, 0x10); // replace with your sensor’s 7-bit address

        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            VL53L5CXGrid.Frame f = tof.readFrame();
            int[] nearest = tof.getNearestCell(f);

            telemetry.addData("Status", f.status);
            telemetry.addData("Frame", f.frameCounter);
            telemetry.addData("Center mm", f.mm[3][3]);
            telemetry.addData("Nearest mm", nearest[2]);
            telemetry.addData("Cell", "(%d,%d)", nearest[0], nearest[1]);
            telemetry.update();

            sleep(50); // 20 Hz
        }
    }
}
