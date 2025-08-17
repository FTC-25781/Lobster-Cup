package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.sensors.VL53L5CXGrid;

@TeleOp(name = "VL53L5CX Demo")
public class VL53Demo extends LinearOpMode {

    // Declare the sensor at the class level
    private VL53L5CXGrid tof;

    @Override
    public void runOpMode() {

        // Initialize the sensor from HardwareMap
        tof = new VL53L5CXGrid(hardwareMap, "tofBridge", 0x10); // replace 0x10 with your sensor’s 7-bit address

        telemetry.addLine("Ready, waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            VL53L5CXGrid.Frame f = tof.readFrame();

            telemetry.addData("Status", f.status);
            telemetry.addData("Frame", f.frameCounter);
            telemetry.addData("Center mm", f.mm[3][3]);

            // Find nearest cell
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

            telemetry.addData("Nearest mm", best);
            telemetry.addData("Cell", "(%d,%d)", bestR, bestC);
            telemetry.update();

            sleep(50); // ~20Hz
        }
    }
}
