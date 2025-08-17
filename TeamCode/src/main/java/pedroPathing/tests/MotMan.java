package pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mot Man Test", group = "Linear OpMode")
public class MotMan extends LinearOpMode {

    private DcMotor motor;
    public double motorTicks;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor = hardwareMap.get(DcMotor.class, "intakeSlides");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {

            if (motor.getCurrentPosition() <= -1050) {
                motorTicks = -1050;
            }

            else if (motor.getCurrentPosition() >= 0) {
                motorTicks = 0;
            }

            else {
                motorTicks=motor.getCurrentPosition();
            }

            if (gamepad1.b) {
                motor.setPower(gamepad1.left_stick_y * 0.2);
            } else {
                motor.setPower(gamepad1.left_stick_y);
            }

            telemetry.addData("Actual Ticks", motorTicks);
            telemetry.addData("Encoder", motor.getCurrentPosition());
            telemetry.addData("Encoder", motor.getCurrentPosition() - (motor.getCurrentPosition()*100/1200));
            telemetry.update();
        }
    }
}
