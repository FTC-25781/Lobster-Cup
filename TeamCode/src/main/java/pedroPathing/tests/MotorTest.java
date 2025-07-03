package pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test", group = "Linear OpMode")
public class MotorTest extends LinearOpMode {
    private DcMotor motor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);
        }
    }
}
